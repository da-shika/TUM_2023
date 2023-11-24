#ifndef SKIN_VISUALIZER_SKIN_VISUALIZER_H_
#define SKIN_VISUALIZER_SKIN_VISUALIZER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include <skin_client/patch_client_factory.h>

#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <control_core/interfaces/module_base.h>
#include <control_core/types.h>
#include <control_core_msgs/SkinPatches.h>

#include <skin_visualizer/wrench_cone_visualizer.h>

namespace skin_visualizer
{
  /**
   * @brief Subscribe to skin skinservers and create useful visualization
   * from the skin data.
   * A merged pointcloud
   * An activation pointcloud
   * A a visulation of clusters and their prinzipal axis
   * 
   */
  class SkinVisualizer : public cc::ModuleBase
  {
    public:
      typedef cc::ModuleBase Base;
      typedef skin_client::SkinPatchClient::Ptr Client;
      typedef std::vector<Client> Clients;
      typedef skin_client::SkinPatchClient::Data Data;

    private:
      std::string tf_prefix_;
      std::string target_frame_;
      std::vector<std::string> configs_;

      // skin patches
      Clients clients_;

      // contacts
      std::vector<cc::SkinPatch> contacts_;

      // transformations
      tf::TransformListener listner_;

      //////////////////////////////////////////////////////////////////////////
      // markers
      //////////////////////////////////////////////////////////////////////////
      visualization_msgs::MarkerArray contact_markers_template_;
      visualization_msgs::MarkerArray contact_markers_;
      visualization_msgs::MarkerArray cell_label_markers_;
      visualization_msgs::MarkerArray cell_line_markers_;

      // visualizations
      WrenchConeVisualizer wrench_cone_vis_;
      
      // pointclouds
      Data::CellCloud::Ptr merged_cell_cloud_;
      Data::DistanceCloud::Ptr merged_dist_cloud_;
      Data::CellCloud::Ptr contact_cloud_;
      std_msgs::Header header_;
      geometry_msgs::PoseArray contact_poses_;

      //////////////////////////////////////////////////////////////////////////
      // ros connections
      //////////////////////////////////////////////////////////////////////////
      ros::Subscriber contacts_sub_;

      ros::Publisher distance_cloud_pub_;     // all distances
      ros::Publisher cell_cloud_pub_;         // all cells
      ros::Publisher contact_cloud_pub_;      // all active cells
      
      ros::Publisher contact_poses_pub_;
      ros::Publisher contact_markers_pub_;
      
      ros::Publisher cell_active_labels_pub_;

    public:
      SkinVisualizer(const std::string& name, const std::string& target_frame, const std::string& tf_prefix);

      virtual ~SkinVisualizer();

      virtual bool init(ros::NodeHandle& nh, Parameters& params) override;

      virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    private:
      void mergePatches(const ros::Time& time);

      void buildMarkers(const ros::Time& time);

    private:
      void skinContactsCallback(const control_core_msgs::SkinPatchesConstPtr& msg);
  };

}

#endif