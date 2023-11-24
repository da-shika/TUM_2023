#include <pinocchio/fwd.hpp>

#include <ics_behavior/behavior_manager.h>

namespace ics
{
  BehaviorManager::BehaviorManager(
      ControllerPtr controller, const std::string& name, VisualizerPtr visualizer, bool verbose) : 
    Base{name},
    controller_{controller},
    visualizer_{visualizer},
    verbose_{verbose}
  {
  }

  BehaviorManager::~BehaviorManager()
  {
  }

  bool BehaviorManager::add(BehaviorPtr behavior, bool active)
  {
    behaviors_[behavior->name()] = behavior;
    if(active)
    {
      active_[behavior->name()] = behavior;
    }
    if(verbose_)
      PRINT_WARN("'%s': Behavior: '%s'", Base::name().c_str(), behavior->name().c_str());
    return true;
  }

  bool BehaviorManager::init(ros::NodeHandle &nh, Parameters &global_params)
  {
    // create the controller
    if(!controller_->initRequest(nh, global_params))
    {
      PRINT_ERROR("Error setting up controller");
      return false;
    }

    // setup the visualizer
    if(visualizer_)
    {
      if(!visualizer_->initRequest(nh, global_params))
      {
        PRINT_ERROR("Error setting up visualizer");
        return false;
      }
    }

    // setup all behaviors and give access to controller
    for(auto behavior : behaviors_)
    {
      behavior.second->setController(controller_);
      if(!behavior.second->initRequest(nh, global_params))
      {
        PRINT_ERROR("'%s': Failed to init behavior: '%s'", 
          Base::name().c_str(), behavior.second->name().c_str());
        return false;
      }
    }

    // load active behaviors
    for(auto behavior : active_)
    {
      if(!behavior.second->loadRequest())
      {
        PRINT_ERROR("'%s': Failed to init behavior: '%s'", 
          Base::name().c_str(), behavior.second->name().c_str());
        return false;
      }
    }

    // setup the ros connections
    change_behaviors_srv_ = 
      nh.advertiseService("change_behavior", &BehaviorManager::changeBehaviorHandler, this);
    list_behaviors_srv_ = 
      nh.advertiseService("list_behavior", &BehaviorManager::listBehaviorHandler, this);
    controller_stopped_pub_ = nh.advertise<std_msgs::Empty>("controller_stopped", 1);
    timing_pub_ = nh.advertise<behavior_msgs::Timing>("controller_timings", 1);
    
    global_params.get("publish_rate", publish_rate_);
    has_change_behavior_ = false;

    ROS_WARN_STREAM("////////////////////////////////////////////////////////");
    ROS_WARN_STREAM("/////////////////// Behaviors //////////////////////////");
    ROS_WARN_STREAM("Loaded Behaviors:");
    for(auto behavior : behaviors_)
      ROS_INFO_STREAM("- " << behavior.first);
    ROS_WARN_STREAM("Active Behaviors:");
    for(auto behavior : active_)
      ROS_INFO_STREAM("- " << behavior.first);
    ROS_INFO_STREAM("////////////////////////////////////////////////////////");

    return true;
  }

  void BehaviorManager::start(const ros::Time &time)
  {
    // start the controller
    controller_->startRequest(time);

    // start visualizer
    if(visualizer_)
      visualizer_->startRequest(time); 

    // set all active state into running
    for(auto behavior : active_)
    {
      behavior.second->startRequest(time);
      if(verbose_)
        PRINT_WARN("'%s': Behavior: '%s'", 
          Base::name().c_str(), behavior.second->name().c_str());
    }

    // start the publisher thread
    if(!Thread::start())
    {
      PRINT_WARN("Publisher Thread failed");
    }
  }

  bool BehaviorManager::update(
    const ros::Time &time, const ros::Duration &period)
  {
    // switch
    if(has_change_behavior_)
    {
      change(time, start_behaviors_, stop_behaviors_);
      start_behaviors_.clear();
      stop_behaviors_.clear();
      has_change_behavior_ = false;
    }

    // update
    bool ok = true;
    for(auto behavior : active_)
    {
      auto start_time = TIMENOW();
      ok &= behavior.second->updateRequest(time, period);
      loop_dur_ = DURATION(start_time);
    }

    // update the controller
    ok &= controller_->updateRequest(time, period);

    return ok;
  }

  void BehaviorManager::stop(const ros::Time &time)
  {
    // force all active states to stop
    for(auto behavior : active_)
    {
      behavior.second->stopRequest(time);
      if(verbose_)
        PRINT_WARN("'%s': Behavior: '%s'", 
          Base::name().c_str(), behavior.second->name().c_str());
    }

    // stop controller
    controller_->stopRequest(time);

    // stop the publisher thread
    if(!Thread::stop())
      PRINT_ERROR("Had to forcefully shutdown publisher thread");
    if(verbose_)
      PRINT_WARN("stopped");

    // inform everyone
    std_msgs::Empty empty_msg;
    controller_stopped_pub_.publish(empty_msg);
  }

  void BehaviorManager::publish(const ros::Time &time)
  {
    // publish behavior states
    for(auto behavior : active_)
      behavior.second->publishRequest(time);

    // publish controller states
    controller_->publishRequest(time);

    // publish default timing outputs
    timing_msg_.solver_dur.data = controller_->formulation().idyn().solverDuration();
    timing_msg_.geometry_dur.data = controller_->formulation().idyn().geometryDuration();
    timing_msg_.control_loop_dur.data = loop_dur_;
    cc::publish_if_subscribed(timing_pub_, timing_msg_);

    // show visualizations
    if(visualizer_)
      visualizer_->updateRequest(time, ros::Duration(0.0));
  }

  void BehaviorManager::run()
  {
    ros::Rate rate(publish_rate_);
    while(ros::ok() && !Thread::hasStopRequest())
    { 
      ros::Time time = ros::Time::now();
      Base::publishRequest(time);
      ros::spinOnce();
      rate.sleep();
    }
    if(verbose_)
      ROS_WARN("BehaviorManager::run() publishing stopped");
  }

  bool BehaviorManager::change(
    const ros::Time &time, 
    const std::vector<std::string>& start,
    const std::vector<std::string>& stop)
  {
    for(auto name : stop)
    {
      auto it = active_.find(name);
      if(it != active_.end())
      {
        BehaviorPtr behavior = it->second;
        behavior->stopRequest(time);
        behavior->unloadRequest();
        active_.erase(it);
      }
      else
      {
        PRINT_WARN("Behavior '%s' not found", name.c_str());
      }
    }

    for(auto name : start)
    {
      auto it = behaviors_.find(name);
      if(it != behaviors_.end())
      {
        BehaviorPtr behavior = it->second;
        behavior->loadRequest();
        behavior->startRequest(time);
        active_[name] = behavior;
      }
      else
      {
        PRINT_WARN("Behavior '%s' not found", name.c_str());
      }
    }
    return true;
  }

  BehaviorManager::BehaviorPtr BehaviorManager::has(const std::string& name)
  {
    auto elem = behaviors_.find(name);
    if(elem == behaviors_.end())
    {
      return nullptr;
    }
    return elem->second;
  }

  bool BehaviorManager::changeBehaviorHandler(
    behavior_msgs::ChangeBehaviorRequest& req,
    behavior_msgs::ChangeBehaviorResponse& res)
  {
    has_change_behavior_ = true;
    start_behaviors_ = req.start_behaviors;
    stop_behaviors_ = req.stop_behaviors;
    res.ok = true;
    return true;
  }

  bool BehaviorManager::listBehaviorHandler(
    behavior_msgs::ListBehaviorRequest& req,
    behavior_msgs::ListBehaviorResponse& res)
  {
    for(auto behavior : active_)
      res.running_behaviors.push_back(behavior.first);
    for(auto behavior : behaviors_)
      res.loaded_behaviors.push_back(behavior.first);
    return true;
  }

}