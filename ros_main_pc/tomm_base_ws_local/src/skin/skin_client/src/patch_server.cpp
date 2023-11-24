#include <skin_client/patch_server.h>

namespace skin_client
{

  SkinPatchServer::SkinPatchServer() : 
    Base()
  {
  }

  SkinPatchServer::~SkinPatchServer()
  {
  }

  bool SkinPatchServer::load(double min_range, double max_range, bool verbose)
  {
    // for now we only use force and proximity
    dynamic_data_.prox.resize(numberOfCells(), 0.0);
    dynamic_data_.force.resize(numberOfCells(), 0.0);
    dynamic_data_.dist.resize(numberOfCells(), max_range);
    return true;
  }

  bool SkinPatchServer::enableDataConnection(ros::NodeHandle& nh)
  {
    if(state_ != LOADED)
    {
      ROS_ERROR("SkinPatchServer::enableDataConnection(): Not loaded");
      return false;
    }

    pub_ = nh.advertise<skin_client::SkinPatchData>(Base::topicName() + "/data", 1);
    return true;
  }

  void SkinPatchServer::publish() const
  {
    if(pub_.getNumSubscribers() > 0)
    {
      pub_.publish(dynamic_data_);
    }
  }

  uint32_t SkinPatchServer::numberOfSubscriber() const
  {
    return pub_.getNumSubscribers();
  }

  const SkinPatchData& SkinPatchServer::dynamicData() const
  {
    return dynamic_data_;
  }

  SkinPatchData& SkinPatchServer::dynamicData()
  {
    return dynamic_data_;
  }

}