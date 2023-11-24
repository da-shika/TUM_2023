#include <pinocchio/fwd.hpp>

#include <ics_behavior/behavior_base.h>

namespace ics
{
  BehaviorBase::BehaviorBase(const std::string& name) : 
    Base{name},
    controller_{nullptr},
    is_loaded_{false},
    has_updated_once_{false}
  {
  }

  BehaviorBase::~BehaviorBase()
  {
  }

  bool BehaviorBase::initRequest(ros::NodeHandle& nh, Base::Parameters& params)
  {
    nh_ = nh;
    params_.addOptional<Tasks>("tasks", Tasks());
    if(!params_.load(nh, Base::name()))
    {
      PRINT_ERROR("'%s' Error loading parameter", Base::name().c_str());
      return false;
    }
    return Base::initRequest(nh, params);
  }
  
  bool BehaviorBase::loadRequest()
  {
    if(!Base::isInitialized())
    {
      PRINT_ERROR("'%s' Not initalized", Base::name().c_str());
      return false;
    }

    if(!controller_)
    {
      PRINT_ERROR("'%s' No controller set", Base::name().c_str());
      return false;
    }

    if(is_loaded_)
    {
      PRINT_ERROR("'%s' is already loaded", Base::name().c_str());
      return true;
    }

    // load the new tasks during runtime
    bool activate = true;
    bool verbose = true;
    if(!controller_->formulation().loadTasksFromParameters(nh_, params_.get<std::vector<std::string> >("tasks"), activate, verbose))
    {
      PRINT_ERROR("BehaviorBase::initRequest(): '%s' Error adding tasks", Base::name().c_str());
      return false;
    }
    is_loaded_ = true;
    return load();
  }

  bool BehaviorBase::startRequest(const ros::Time &time)
  {
    if (!is_loaded_ || !Base::moduleSetStartState(ros::Time::now()))
    {
      PRINT_ERROR("'%s' called but not initalized.", Base::name().c_str());
      return false;
    }
    has_updated_once_ = false;
    return start(time);
  }

  bool BehaviorBase::updateRequest(const ros::Time &time, const ros::Duration &period)
  {
    if (!Base::isRunning())
    {
      PRINT_ERROR("'%s' called but not running.", Base::name().c_str());
      return false;
    }
    bool ret = update(time, period);
    has_updated_once_ = true;
    return ret;
  }

  bool BehaviorBase::stopRequest(const ros::Time &time)
  {
    if (Base::moduleSetStopState(ros::Time::now()))
    {
      stop(time);
      return true;
    }
    return false;
  }

  bool BehaviorBase::unloadRequest()
  {
    if(Base::isRunning())
    {
      PRINT_ERROR("'%s' called but still running.", Base::name().c_str());
      return false;
    }

    if(!is_loaded_)
    {
      ROS_WARN("'%s' is already unloaded", Base::name().c_str());
      return true;
    }

    if(!controller_->formulation().unLoadTasks(params_.get<std::vector<std::string> >("tasks"), true))
    {
      PRINT_ERROR("'%s' error removing tasks.", Base::name().c_str());
      return false;
    }
    is_loaded_ = false;
    return unload();
  }

  void BehaviorBase::publishRequest(const ros::Time &time)
  {
    if(has_updated_once_)
      publish(time);
  }

}