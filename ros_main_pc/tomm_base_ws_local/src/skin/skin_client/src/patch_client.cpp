#include <skin_client/patch_client.h>

namespace skin_client
{

  SkinPatchClient::VectorRGB SkinPatchClient::min_dist_color_(255,0,0);
  SkinPatchClient::VectorRGB SkinPatchClient::max_dist_color_(0,255,0);

  SkinPatchClient::SkinPatchClient() : 
    Base(),
    has_connection_(false)
  {
  }

  SkinPatchClient::~SkinPatchClient()
  {
  }

  bool SkinPatchClient::load(double min_range, double max_range, bool verbose)
  {
    // setup measurments
    data_.proximity.resize(Base::numberOfCells(), 0.0);
    data_.force.resize(Base::numberOfCells(), 0.0);
    data_.distance.resize(Base::numberOfCells(), max_range);

    // setup pointclouds
    data_.distance_cloud.reset(new Data::DistanceCloud());
    data_.distance_cloud->resize(Base::numberOfCells());
    data_.cell_cloud.reset(new Data::CellCloud());
    data_.cell_cloud->resize(Base::numberOfCells());
    
    // resever memory for active (components)
    data_.active_cell_cloud.reset(new Data::CellCloud());
    data_.active_cell_cloud->reserve(Base::numberOfCells());
    data_.active_id.reserve(Base::numberOfCells());
    data_.active_distance.reserve(Base::numberOfCells());
    data_.active_proximity.reserve(Base::numberOfCells());
    data_.active_force.reserve(Base::numberOfCells());

    // init pointclouds
    size_t k = 0;
    for(auto it = data_.cell_cloud->begin(); it != data_.cell_cloud->end(); ++it, ++k)
    {
      it->getArray3fMap() = data_.positions.col(k).cast<float>();
      it->getNormalVector3fMap() = data_.normals.col(k).cast<float>();
    }
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    for(auto it = data_.distance_cloud->begin(); it != data_.distance_cloud->end(); ++it)
      it->x = it->y = it->z = invalid;
    data_.cell_cloud->header.frame_id = Base::jointFrame();
    data_.distance_cloud->header.frame_id = Base::jointFrame();
  
    return true;
  }

  bool SkinPatchClient::hasConnection() const
  {
    return has_connection_;
  }

  bool SkinPatchClient::enableDataConnection(ros::NodeHandle& nh)
  {
    if(state_ != LOADED)
    {
      ROS_ERROR("SkinPatchClient::enableDataConnection(): Not loaded");
      return false;
    }
    sub_ = nh.subscribe(Base::topicName() + "/data", 1, &SkinPatchClient::callback, this);
    return true;
  }

  void SkinPatchClient::computePointCloud()
  {
    float factor;
    Eigen::Vector3f s;
    Eigen::Vector3i c;

    size_t k = 0;
    auto dist_it = data_.distance_cloud->begin();
    auto cell_it = data_.cell_cloud->begin();
    for(; cell_it != data_.cell_cloud->end(); ++dist_it, ++cell_it, ++k)
    {
      // set the position
      s = cell_it->getArray3fMap();
      dist_it->getArray3fMap() = 
        float(data_.distance[k]) * data_.normals.col(k).cast<float>() + s;

      // set the color based on distance
      factor = std::min(1.0, std::max(0.0, (0.06 - data_.distance[k])/0.06));
      dist_it->r = factor*min_dist_color_[0] + (1.-factor)*max_dist_color_[0];
      dist_it->g = factor*min_dist_color_[1] + (1.-factor)*max_dist_color_[1];
      dist_it->b = factor*min_dist_color_[2] + (1.-factor)*max_dist_color_[2];
    }
  }

  void SkinPatchClient::extractActiveInformation(const Data::Indices& active)
  {
    data_.active_proximity.resize(active.size());
    data_.active_force.resize(active.size());
    data_.active_distance.resize(active.size());
    data_.active_id.resize(active.size());
    data_.active_cell_cloud->points.resize(active.size());

    int i = 0;
    for(auto it = active.begin(); it != active.end(); ++it, ++i)
    {
      data_.active_proximity[i] = data_.proximity[*it];
      data_.active_force[i] = data_.force[*it];
      data_.active_distance[i] = data_.distance[*it];
      data_.active_id[i] = data_.ids[*it];
      data_.active_cell_cloud->at(i) = data_.cell_cloud->at(*it);
    }
  }

  SkinPatchClient::Data::Indices SkinPatchClient::computeActiveDistanceIndices(double thresh)
  {
    Data::Indices indices;
    for(int i = 0; i < data_.distance.size(); ++i)
      if(data_.distance[i] < thresh)
        indices.push_back(i);
    return indices;
  }

  SkinPatchClient::Data::Indices SkinPatchClient::computeActiveProximityIndices(double thresh)
  {
    Data::Indices indices;
    for(int i = 0; i < data_.proximity.size(); ++i)
      if(data_.proximity[i] > thresh)
        indices.push_back(i);
    return indices;
  }

  SkinPatchClient::Data::Indices SkinPatchClient::computeActiveForceIndices(double thresh)
  {
    Data::Indices indices;
    for(int i = 0; i < data_.force.size(); ++i)
      if(data_.force[i] > thresh)
        indices.push_back(i);
    return indices;
  }

  void SkinPatchClient::callback(const skin_client::SkinPatchDataConstPtr& msg)
  {
    has_connection_ = true;
    if(data_.proximity.size() == msg->prox.size())
      data_.proximity = msg->prox;
    if(data_.force.size() == msg->force.size())
      data_.force = msg->force;
    if(data_.distance.size() == msg->dist.size())
      data_.distance = msg->dist;
  }

}