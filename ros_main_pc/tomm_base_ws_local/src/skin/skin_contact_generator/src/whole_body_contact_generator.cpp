#include <skin_contact_generator/whole_body_contact_generator.h>
#include <skin_contact_generator/patch_computations.h>

////////////////////////////////////////////////////////////////////////////////
// pcl includes
////////////////////////////////////////////////////////////////////////////////
#include <pcl/cloud_iterator.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/math/quaternion.h>
#include <control_core/ros/marker.h>
#include <control_core/ros/ros.h>

#include <std_msgs/Int64.h>

namespace skin_contact_generator
{

  WholeBodyContactGenerator::WholeBodyContactGenerator(const std::string& name) : 
    Base(name)
  {
  }

  WholeBodyContactGenerator::~WholeBodyContactGenerator()
  {
  }

  bool WholeBodyContactGenerator::init(ros::NodeHandle& nh, Base::Parameters& global_params)
  {
    //////////////////////////////////////////////////////////////////////////
    // load parameters
    //////////////////////////////////////////////////////////////////////////
    if(!params_.fromParamServer(nh, Base::name()))
    {
      PRINT_ERROR("Error loading parameters");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // connect to all available skin patches
    ////////////////////////////////////////////////////////////////////////////

    // setup clients (in robot namespace)
    std::string ns;
    if(skin_client::split(nh.getNamespace(), '/').size() > 1)
      ns = skin_client::split(nh.getNamespace(), '/')[1];

    ROS_WARN_STREAM("ns=" << ns);
    auto ret = skin_client::list_available_patches(ns);
    for(const auto str : ret)
      ROS_WARN_STREAM(str);



    size_t total_number_cells;
    Clients client_candidates;
    if(!skin_client::load_clients(client_candidates, total_number_cells, ns, true))
    {
      ROS_ERROR("WholeBodyContactGenerator::initialize(): No servers found");
      return false;
    }

    // sort patches based on their reference frame
    std::unordered_map<std::string, Clients> frame_map;
    for(auto client : client_candidates)
    {
      if(cc::has(params_.masked_patches, client->name()))
      {
        PRINT_WARN("'%s' masked",client->name().c_str()); 
        continue;
      }
      frame_map[client->jointFrame()].push_back(client);
      clients_.push_back(client);
    }

    // connect patches that have the same reference frame
    for(auto it = frame_map.begin(); it != frame_map.end(); ++it)
    {
      SkinPatchMap map;
      map.frame = it->first;
      map.clients = it->second;
      skinpatch_maps_.push_back(map);
    }

    // start all client connections
    for(auto client : clients_)
    {
      client->enableDataConnection(nh);
    }

    ////////////////////////////////////////////////////////////////////////////
    // ros connections
    ////////////////////////////////////////////////////////////////////////////
    contact_pub_ = nh.advertise<control_core_msgs::SkinPatches>("whole_body_contact/skin_contacts", 1);
    n_active_cell_pub_ = nh.advertise<control_core_msgs::VectorStamped>("whole_body_contact/n_active_cell", 1);
    
    return true;
  }

  bool WholeBodyContactGenerator::update(const ros::Time &time, const ros::Duration &period)
  {
    ////////////////////////////////////////////////////////////////////////////
    // loacted on the same robot frame: compute contacts
    ////////////////////////////////////////////////////////////////////////////
    n_active_cells_ = 0;
    contacts_.clear();
    for(size_t i = 0; i < skinpatch_maps_.size(); ++i)
    {
      const auto& map = skinpatch_maps_[i];

      //////////////////////////////////////////////////////////////////////////
      // extract information of active cells only
      //////////////////////////////////////////////////////////////////////////
      
      auto active_cells = Data::CellCloud::Ptr(new Data::CellCloud());
      Data::Indices active_id_data;
      Data::Measurement active_force_data;
      Data::Measurement active_proximity_data;
      Data::Measurement active_distance_data;

      // check each patch for active cells, extract indices, force and proximity
      Data::Indices indices;
      for(auto client : map.clients)
      {
        indices = client->computeActiveDistanceIndices(params_.active_distance);
        client->extractActiveInformation(indices);

        // store active data
        cc::insert(client->data().active_force, active_force_data);
        cc::insert(client->data().active_proximity, active_proximity_data);
        cc::insert(client->data().active_distance, active_distance_data);
        cc::insert(client->data().active_id, active_id_data);
        active_cells->insert(active_cells->end(), client->data().active_cell_cloud->begin(), client->data().active_cell_cloud->end());
      }      
      if(active_proximity_data.empty())
        continue;
        
      //////////////////////////////////////////////////////////////////////////
      // compute their clusters in active_cells
      //////////////////////////////////////////////////////////////////////////

      pcl::search::KdTree<Data::CellPoint>::Ptr tree (new pcl::search::KdTree<Data::CellPoint>);
      tree->setInputCloud(active_cells);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<Data::CellPoint> ec;
      ec.setClusterTolerance(params_.cluster_tolerance);
      ec.setMinClusterSize(params_.cluster_min_size);
      ec.setMaxClusterSize(params_.cluster_max_size);
      ec.setSearchMethod(tree);
      ec.setInputCloud(active_cells);
      ec.extract(cluster_indices);

      //////////////////////////////////////////////////////////////////////////
      // compute contact data
      //////////////////////////////////////////////////////////////////////////

      std::vector<pcl::PointIndices>::const_iterator cit = cluster_indices.begin();
      for(; cit != cluster_indices.end(); ++cit) 
      {
        // create a new contact
        SkinContact contact;
        contact.patch.frame() = map.frame;
        contact.patch_map_id = i;
        contact.ids = extractCellIds(cit->indices, active_id_data);             // test use cell ids as identifier

        // create contact frame
        computeContactFrame(*active_cells, cit->indices, contact.patch.pose());

        // compute the wrenches acting at that contact, base oriented
        computeContactWrench(
          *active_cells, cit->indices, active_force_data, active_proximity_data, 
          active_distance_data, CONTACT, contact.patch);
        
        // compute the contact hulls
        computeContactHull(*active_cells, cit->indices, CONTACT, contact.patch);

        // store contact
        contacts_.push_back(contact);
      }

      n_active_cells_ += active_cells->size();
    }

    ////////////////////////////////////////////////////////////////////////////
    // publish the information
    ////////////////////////////////////////////////////////////////////////////
    control_core_msgs::SkinPatches msg;
    for(const auto& contact : contacts_)
      msg.patches.push_back(contact.patch);
    cc::publish_if_subscribed(contact_pub_, msg);
    
    control_core_msgs::VectorStamped n_active_cell_msgs;
    n_active_cell_msgs.header.stamp = time;
    n_active_cell_msgs.vector.data.push_back(n_active_cells_);
    cc::publish_if_subscribed(n_active_cell_pub_, n_active_cell_msgs);

    return true;
  }

  WholeBodyContactGenerator::Data::Indices WholeBodyContactGenerator::extractCellIds(
    const Data::Indices indices, const Data::Indices& ids)
  {
    Data::Indices extract;
    extract.reserve(indices.size());
    for(auto idx : indices)
    {
      extract.push_back(ids[idx]);
    }
    return extract;
  }

}