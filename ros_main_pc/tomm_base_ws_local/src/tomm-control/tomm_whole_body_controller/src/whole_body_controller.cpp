#include <pinocchio/fwd.hpp>

#include <tomm_whole_body_controller/whole_body_controller.h>

#include <control_core/test_utilities/timing.h>

namespace tomm
{

  WholeBodyController::WholeBodyController(
      HardwareInterface& hardware_interface, const std::string& name) : 
    Base(name),
    hardware_interface_(hardware_interface)
  {
  }

  WholeBodyController::~WholeBodyController()
  {
  }

  bool WholeBodyController::init(ros::NodeHandle& nh, cc::Parameters& global_params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // parameter
    ////////////////////////////////////////////////////////////////////////////
    // if(!params_.fromParamServer(nh, Base::name()))
    // {
    //   PRINT_ERROR("Can't load parameter");
    //   return false;
    // }
    // reconfig_srv_ = std::make_unique<Server>(params_.privateNamespace());
    // reconfig_srv_->setCallback(boost::bind(&WholeBodyController::reconfigureRequest, this, _1, _2));

    ////////////////////////////////////////////////////////////////////////////
    // commanded robot
    ////////////////////////////////////////////////////////////////////////////
    cmd_robot_ = std::make_unique<CommandedRobot>("formulation");
    if(!cmd_robot_->initRequest(nh, global_params))
    {
      PRINT_ERROR("Error init CommandedRobot");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // real robot
    ////////////////////////////////////////////////////////////////////////////
    // real_robot_ = std::make_unique<RealRobot>(hardware_interface_, *cmd_robot_, "robot");
    // if(!real_robot_->initRequest(nh, global_params))
    // {
    //   PRINT_ERROR("Error init RealRobot");
    //   return false;
    // }

    return true;
  }

  void WholeBodyController::start(const ros::Time &time)
  {
    // update the commanded robot with current real jointstate
    // note: this projects robot on the ground and recomputes active task references
    cmd_robot_->start(hardware_interface_.jointState(), time);
    PRINT_INFO_STREAM("cmd_robot:\n" << cmd_robot_->toString());

    // start the real robot
    // note: sets intial fb estimation to cmd robot fb state
    // real_robot_->startRequest(time);
    // PRINT_INFO_STREAM("real_robot:\n" << real_robot_->toString());

    command_ = cmd_robot_->jointState();
    PRINT_WARN_STREAM("inital command:\n" << command_.toString());
  }

  bool WholeBodyController::update(const ros::Time &time, const ros::Duration &period)
  {
    bool ret = true;

    ////////////////////////////////////////////////////////////////////////////
    // read system states
    ////////////////////////////////////////////////////////////////////////////

    // optain real robot state
    // real_robot_->update(balancer_->walkingStates(), time, period);
    // const auto& real_state = real_robot_->robotState();
    // obtain commanded robot state
    const auto& cmd_state = cmd_robot_->robotState();

    ////////////////////////////////////////////////////////////////////////////
    // update formulation
    //////////////////////////////////////////////////////////////////////////

    // solve formulation with fitered real state input and integrate solution
    ret &= cmd_robot_->update(cmd_state.joints(), time, period);

    ////////////////////////////////////////////////////////////////////////////
    // finalize robot command
    ////////////////////////////////////////////////////////////////////////////
    command_ = cmd_robot_->jointState();
      
    if(!ret)
    {
      // print the last known state of every module
      PRINT_ERROR_STREAM("Controller Failure state:\n"
        << " Commanded Robot:\n " << cmd_robot_->toString() << "\n"
        << " Hardware:\n" << hardware_interface_.toString());
    }
    return ret;
  }

  void WholeBodyController::stop(const ros::Time &time)
  {
  }

  void WholeBodyController::publish(const ros::Time &time)
  {
    cmd_robot_->formulation().idyn().robot().broadcastFrames(time);
  }

  // void WholeBodyController::reconfigureRequest(Config &config, uint32_t level)
  // {
  //   params_.updateConfig(config, level);
  // }

}