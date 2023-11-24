#include <skin_visualizer/skin_visualizer.h>
#include <skin_visualizer/utilities.h>

#include <pcl/cloud_iterator.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <control_core/math/quaternion.h>
#include <control_core/ros/marker.h>
#include <control_core/math/transform.h>

namespace skin_visualizer
{

  SkinVisualizer::SkinVisualizer(const std::string& name, const std::string& target_frame, const std::string& tf_prefix) : 
    Base{name},
    target_frame_{target_frame},
    tf_prefix_{tf_prefix}
  {
  }

  SkinVisualizer::~SkinVisualizer()
  {
  }

  bool SkinVisualizer::init(ros::NodeHandle& nh, Parameters& params)
  {
    // handle tf prefix
    target_frame_ = tf_prefix_ + "/" + target_frame_;
    contact_poses_.header.frame_id = target_frame_;
    header_.frame_id = target_frame_;

    // setup clients (in robot namespace)
    std::string ns;
    if(skin_client::split(nh.getNamespace(), '/').size() > 1)
      ns = skin_client::split(nh.getNamespace(), '/')[1];

    size_t total_number_cells;
    if(!skin_client::load_clients(clients_, total_number_cells, ns, true))
    {
      ROS_ERROR("SkinVisualizer::initialize(): No servers found");
      return false;
    }

    // start all client connections
    for(auto client : clients_)
    {
      client->enableDataConnection(nh);
      // set tf prefix
      client->data().cell_cloud->header.frame_id.insert(0, tf_prefix_ + "/");
      client->data().distance_cloud->header.frame_id.insert(0, tf_prefix_ + "/");
    }

    // pointclouds
    merged_cell_cloud_.reset(new Data::CellCloud());
    merged_cell_cloud_->points.reserve(total_number_cells);
    merged_dist_cloud_.reset(new Data::DistanceCloud());
    merged_dist_cloud_->points.reserve(total_number_cells);
    contact_cloud_.reset(new Data::CellCloud());
    merged_dist_cloud_->points.reserve(total_number_cells);

    // build markers
    contact_markers_template_.markers.push_back(cc::create_arrow_marker("wrench", 1, 0.02, 0.03, 1, 0, 0, target_frame_));
    contact_markers_template_.markers.push_back(cc::create_arrow_marker("wrench", 2, 0.02, 0.03, 0, 1, 0, target_frame_));
    contact_markers_template_.markers.push_back(cc::create_sphere_marker("cop", 3, 0.025, 1, 0, 0, target_frame_));
    contact_markers_template_.markers.push_back(cc::create_sphere_marker("cop", 4, 0.025, 0, 1, 0, target_frame_));
    contact_markers_template_.markers.push_back(cc::create_line_strip_marker("hull", 5, 0.01, 0.0, 0.0, 1.0, target_frame_));
    for(size_t i = 0; i < total_number_cells; ++i)
    {
      cell_label_markers_.markers.push_back(cc::create_text_marker("cell_label", i, 0.03, 1.0, 0.0, 0.0, target_frame_));
      cell_label_markers_.markers.back().text = ".";
    }

    for(auto &marker : contact_markers_template_.markers)
      marker.lifetime = ros::Duration(0.1);

    // ros connections
    contacts_sub_ = nh.subscribe("/h1/whole_body_contact/skin_contacts", 1, &SkinVisualizer::skinContactsCallback, this);

    // pointclouds
    distance_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("distance_cloud_merged", 1);
    cell_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cell_cloud_merged", 1);
    contact_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("contact_cloud", 1);

    // contact visualiztions
    contact_poses_pub_ = nh.advertise<geometry_msgs::PoseArray>("contact_poses", 1);
    contact_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("contact_marker", 1);
    
    // cell visualizations
    cell_active_labels_pub_ = nh.advertise<visualization_msgs::MarkerArray>("cell_active_labels", 1);

    return true;
  }

  bool SkinVisualizer::update(const ros::Time &time, const ros::Duration &period)
  {
    // merge and publish all clouds
    mergePatches(time);
    cell_cloud_pub_.publish(merged_cell_cloud_);
    distance_cloud_pub_.publish(merged_dist_cloud_);
    cell_active_labels_pub_.publish(cell_label_markers_);
    contact_cloud_pub_.publish(contact_cloud_);

    // publish obstacles
    // buildMarkers(time);
    // contact_markers_pub_.publish(contact_markers_);
    // contact_poses_pub_.publish(contact_poses_);

    return true;
  }

  void SkinVisualizer::mergePatches(const ros::Time& time)
  {
    Data::CellCloud transf_cell_cloud;
    Data::DistanceCloud transf_dist_cloud;

    merged_cell_cloud_->clear();
    merged_dist_cloud_->clear();
    contact_cloud_->clear();
    
    size_t k = 0;
    for(auto client : clients_)
    {
      client->computePointCloud();
      listner_.waitForTransform(tf_prefix_+"/"+client->jointFrame(), target_frame_, ros::Time(0), ros::Duration(0.1));

      //////////////////////////////////////////////////////////////////////////
      // point clouds
      //////////////////////////////////////////////////////////////////////////
      pcl_ros::transformPointCloudWithNormals(target_frame_, *client->data().cell_cloud, transf_cell_cloud, listner_);
      pcl_ros::transformPointCloud(target_frame_, *client->data().distance_cloud, transf_dist_cloud, listner_);
      merged_cell_cloud_->insert(merged_cell_cloud_->end(), transf_cell_cloud.begin(), transf_cell_cloud.end());
      merged_dist_cloud_->insert(merged_dist_cloud_->end(), transf_dist_cloud.begin(), transf_dist_cloud.end());
    }

    header_.stamp = ros::Time::now();
    pcl_conversions::toPCL(header_, merged_cell_cloud_->header);
    pcl_conversions::toPCL(header_, merged_dist_cloud_->header);
    pcl_conversions::toPCL(header_, contact_cloud_->header);
  }

  void SkinVisualizer::buildMarkers(const ros::Time& time)
  {
    ////////////////////////////////////////////////////////////////////////////
    // frames
    ////////////////////////////////////////////////////////////////////////////
    contact_poses_.poses.clear();

    tf::StampedTransform tf_transform;
    cc::CartesianPosition X_patch_target, X_pose_target;
    for(const auto& contact : contacts_)
    {
      listner_.lookupTransform(target_frame_, contact.frame(), ros::Time(0), tf_transform);
      X_patch_target = tf_transform;
      X_pose_target = X_patch_target * contact.pose();
      contact_poses_.poses.push_back(X_pose_target);
    }
    contact_poses_.header.stamp = time;                        

    ////////////////////////////////////////////////////////////////////////////
    // markers
    ////////////////////////////////////////////////////////////////////////////
    contact_markers_.markers.clear();

    cc::LinearPosition p1;
    size_t k = 0;
    for(const auto& contact : contacts_)
    {
      // contact frame
      auto pose = contact.pose();

      // force vector
      auto& force_marker = contact_markers_template_.markers[0];
      force_marker.header.stamp = ros::Time(0);
      force_marker.points.resize(2);
      force_marker.points[0] = cc::changeRefFrame(contact.force().cop(), pose);
      p1 = contact.force().cop() + 0.4*contact.force().wrench().force();
      force_marker.points[1] = cc::changeRefFrame(p1, pose);
      force_marker.id = k;
      force_marker.header.frame_id = contact.frame();
      contact_markers_.markers.push_back(force_marker);

      // proximity vector
      auto& proximity_marker = contact_markers_template_.markers[1];
      proximity_marker.header.stamp = ros::Time(0);
      proximity_marker.points.resize(2);
      proximity_marker.points[0] = cc::changeRefFrame(contact.proximity().cop(), pose);
      p1 = contact.proximity().cop() + 0.4*contact.proximity().wrench().force();
      proximity_marker.points[1] = cc::changeRefFrame(p1, pose);
      proximity_marker.id = k+1;
      proximity_marker.header.frame_id = contact.frame();
      contact_markers_.markers.push_back(proximity_marker);

      // force center of preasure
      auto& force_cop_marker = contact_markers_template_.markers[2];
      force_cop_marker.header.stamp = ros::Time(0);
      force_cop_marker.pose.position = cc::changeRefFrame(contact.force().cop(), pose);
      force_cop_marker.id = k+2;
      force_cop_marker.header.frame_id = contact.frame();
      contact_markers_.markers.push_back(force_cop_marker);

      // proximity center of preasure
      auto& proximity_cop_marker = contact_markers_template_.markers[3];
      proximity_cop_marker.header.stamp = ros::Time(0);
      proximity_cop_marker.pose.position = cc::changeRefFrame(contact.proximity().cop(), pose);
      proximity_cop_marker.id = k+3;
      proximity_cop_marker.header.frame_id = contact.frame();
      contact_markers_.markers.push_back(proximity_cop_marker);

      // hull marker
      auto& hull_marker = contact_markers_template_.markers[4];
      hull_marker.header.stamp = ros::Time(0);
      hull_marker.id = k+4;
      hull_marker.header.frame_id = contact.frame();

      cc::Vector3 p_c, p_j; geometry_msgs::Point point;
      auto& vertices = contact.force().hull().vertices();
      hull_marker.points.clear();
      for(size_t i = 0; i < vertices.cols(); ++i)
      {
        p_c << vertices.col(i), 0.0;
        p_j = contact.pose() * p_c;
        point = p_j;
        hull_marker.points.push_back(point);
      }
      point = hull_marker.points[0];
      hull_marker.points.push_back(point);
      contact_markers_.markers.push_back(hull_marker);

      // cone marker
      wrench_cone_vis_.compute(contact, 0.3);
      auto cone = wrench_cone_vis_.toMarkerMsg(0.0, 1.0, 0.0, 0.4);
      cone.id = k+5;
      cone.header.stamp = ros::Time(0);
      cone.header.frame_id = contact.frame();
      contact_markers_.markers.push_back(cone);

      // check limits
      k += 6;
    }
  }

  void SkinVisualizer::skinContactsCallback(const control_core_msgs::SkinPatchesConstPtr& msg)
  {
    contacts_.resize(msg->patches.size());
    for(size_t i = 0; i < msg->patches.size(); ++i)
      contacts_[i] = msg->patches[i];
  }

}