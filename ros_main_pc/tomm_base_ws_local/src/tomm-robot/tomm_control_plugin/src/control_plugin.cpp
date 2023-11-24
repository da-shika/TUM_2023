#include <pinocchio/fwd.hpp>

#include <tomm_control_plugin/control_plugin.h>
#include <tomm_control_plugin/utilities.h>

#include <tomm_core/GlobalParameters.h>

#include <control_core/ros/ros.h>
#include <control_core/test_utilities/timing.h>

namespace tomm_control_plugin
{

  TOMMControlPlugin::TOMMControlPlugin() : 
    Base{true} // true
  {
  }

  TOMMControlPlugin::~TOMMControlPlugin()
  {
  }

  bool TOMMControlPlugin::internalInit(
    ros::NodeHandle &root_nh,
    ros::NodeHandle &controller_nh,
    ClaimedResources &claimed_resources)
  {
    ros::NodeHandle nh("/tomm");                                                // TODO: Use plugin namespace?

    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////
    tomm_core::GlobalParameters global_params;
    if(!global_params.load(nh))
    {
      ROS_ERROR("TOMMControlPlugin::internalInit(): Cannot load parameters");
      return false;
    }
    period_.fromSec(1./global_params.loop_rate);

    ////////////////////////////////////////////////////////////////////////////
    // Setup plugin mode
    ////////////////////////////////////////////////////////////////////////////
    if (global_params.simulation)
    {
      ROS_INFO("Plugin '%s' configured for Gazebo Simulation", 
        controller_nh.getNamespace().c_str());

      // load gazebo sim freq
      if(!cc::load("/gazebo/time_step", sim_dt_))
      {
        ROS_ERROR("TOMMControlPlugin::internalInit(): Can not find parameter '/gazebo/time_step'");
        return false;
      }
      sim_step_factor_ = int(1e6/(sim_dt_*global_params.loop_rate));
    }
    else
    {
      ROS_INFO("Plugin '%s' configured for real Robot", 
        controller_nh.getNamespace().c_str());
      sim_step_factor_ = 1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // setup connection
    ////////////////////////////////////////////////////////////////////////////
    comm_interface_ = std::make_unique<tomm::CommInterface>("robot_comm");
    if(!comm_interface_->initRequest(nh, global_params))
    {
      ROS_ERROR("TOMMControlPlugin::internalInit() Error init comm_interface");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // setup inital robot state
    ////////////////////////////////////////////////////////////////////////////
    size_t na = joint_handles_.size();

    // set joint state and command to last command
    auto& cmd = comm_interface_->command();
    cmd.setZero(na);
    for(size_t i = 0; i < na; ++i)
    {
      if(comm_interface_->state() == tomm::CommInterface::CONSTRUCTED)
        cmd.pos()[i] = joint_handles_[i]->getPosition();
      else
        cmd.pos()[i] = joint_handles_[i]->getCommandPosition();
      cmd.vel()[i] = 0.0;
    }

    // robot state
    auto& robot_state = comm_interface_->robotState();
    robot_state.setZero(na);
    robot_state.joints() = cmd;

    // ft sensors
    robot_state.ftSensors().resize(4, cc::FtSensor::Zero());
    // for(size_t i = 0; i < ft_handles_.size(); ++i)
    //   robot_state.ftSensors()[i].frame() = ft_handles_[i]->getFrameId();

    // read once
    read(joint_handles_, ft_handles_, robot_state);

    ////////////////////////////////////////////////////////////////////////////
    // setup inital app
    ////////////////////////////////////////////////////////////////////////////
    
    app_ = std::make_unique<tomm::App>(*comm_interface_, "app");
    if(!app_->initRequest(nh, global_params))
    {
      ROS_ERROR("TOMMControlPlugin::internalInit(): Error initalizing app");
      return false;
    }

    ROS_WARN("TOMMControlPlugin::internalInit() init '%s'", Base::name().c_str());
    return true;
  }

  void TOMMControlPlugin::internalStarting(const ros::Time &time)
  {
    prev_time_ = time;
    start_time_ = time;

    ////////////////////////////////////////////////////////////////////////////
    // start
    ////////////////////////////////////////////////////////////////////////////
    comm_interface_->startRequest(time);
    app_->startRequest(time);

    start_joint_pos_ =  comm_interface_->command().pos();

    ROS_WARN("TOMMControlPlugin::internalStarting() start '%s'", Base::name().c_str());
  }

  void TOMMControlPlugin::internalUpdate(const ros::Time &time, const ros::Duration &period)
  {
    if ((time.nsec % sim_step_factor_) == 0)
    {
      ros::Duration dt = time - prev_time_;
      prev_time_ = time;
      controllerUpdate(time, period_);
    }
    ros::spinOnce();
  }

  void TOMMControlPlugin::controllerUpdate(const ros::Time &time, const ros::Duration &period)
  {
    auto start_time = TIMENOW();

    ////////////////////////////////////////////////////////////////////////////
    // update the controller
    ////////////////////////////////////////////////////////////////////////////
    
    // read the new state into comm interface
    read(joint_handles_, ft_handles_, comm_interface_->robotState());
    
    // update comm interface
    if(!comm_interface_->updateRequest(time, period))
    {
      ROS_ERROR("TOMMControlPlugin::controllerUpdate: Communication failure");
      stopRequest(time);
    }

    // update controller
    if(!app_->updateRequest(time, period))
    {
      ROS_ERROR("TOMMControlPlugin::controllerUpdate: Controller '%s' failure:\n %s", Base::name().c_str(), 
        print_handles(joint_handles_, ft_handles_).c_str());

      // set everything to zero
      comm_interface_->command().pos() = comm_interface_->robotState().joints().pos();
      comm_interface_->command().vel().setZero();
      comm_interface_->command().acc().setZero();
      write(comm_interface_->command(), joint_handles_);

      stopRequest(time);
    }

    // send the command to the robot
    write(comm_interface_->command(), joint_handles_);

    // print timing
    auto loop_time = DURATION(start_time);
    ROS_INFO_STREAM_THROTTLE(1.0, "TOMMControlPlugin::controllerUpdate: update time=" << loop_time/1000 << " us");
  }

  void TOMMControlPlugin::internalStopping(const ros::Time &time)
  {
    // inform client about shutdown
    app_->stopRequest(time);
    comm_interface_->stopRequest(time);
    ROS_WARN("H1RemoteControlPlugin::internalStopping() stop '%s'", Base::name().c_str());
  }
  
}