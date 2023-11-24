#include <pinocchio/fwd.hpp>

#include <tomm_application/app.h>

//////////////////////////////////////////////////////////////////////////
// ics_tsid_tasks inclues
//////////////////////////////////////////////////////////////////////////
#include <ics_tsid_tasks/tasks.h>
#include <ics_behavior/composite_behavior.h>

////////////////////////////////////////////////////////////////////////////////
// ics basic behaviors
////////////////////////////////////////////////////////////////////////////////
#include <ics_basic_behaviors/move_to_cartesian.h>
#include <ics_basic_behaviors/move_to_joint.h>
#include <ics_basic_behaviors/teleop_frame.h>
#include <ics_basic_behaviors/teleop_joints.h>

////////////////////////////////////////////////////////////////////////////////
// tomm basic behaviors
////////////////////////////////////////////////////////////////////////////////
#include <tomm_basic_behaviors/move_base.h>
#include <tomm_basic_behaviors/move_to_cartesian_mobile_manipulator.h>
  
namespace tomm
{

  App::App(tomm::CommunicationInterface& comm_interface, const std::string& name) : 
    Base{name},
    comm_interface_{comm_interface}
  {
    ics::register_task_types();
  }

  App::~App()
  {
  }

  bool App::init(ros::NodeHandle& nh, cc::Parameters& params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////
    if(!params_.fromParamServer(nh))
    {
      PRINT_ERROR("Error loading parameters");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Setup h1 hardware interface
    ////////////////////////////////////////////////////////////////////////////
    hardware_interface_ = std::make_unique<tomm::TOMMInterface>(comm_interface_, "hardware_interface");
    if(!hardware_interface_->initRequest(nh, params_))
    {
      PRINT_ERROR("Failed to initialize humanoid harware interface");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Setup controller
    ////////////////////////////////////////////////////////////////////////////
    controller_ = std::make_shared<tomm::WholeBodyController>(
      *hardware_interface_, "whole_body_controller");
      
    //visualizer_ = std::make_shared<tomm::Visualizer>(*controller_, "visualizer");

    behavior_manager_ = std::make_unique<ics::BehaviorManager>(
      controller_, "behavior_manager", nullptr);

    ////////////////////////////////////////////////////////////////////////////
    // Setup behaviors
    ////////////////////////////////////////////////////////////////////////////

    // teleoperation
    behavior_manager_->add(
        std::make_shared<ics::CompositeBehavior>("teleop", 
          ics::CompositeBehavior::Composites{
            //std::make_shared<ics::TeleopFrame>("teleop_head"),
            std::make_shared<ics::TeleopFrame>("teleop_left_hand"),
            std::make_shared<ics::TeleopFrame>("teleop_right_hand")}), false);

    behavior_manager_->add(std::make_shared<ics::TeleopJoints>("teleop_joints"), false);

    // move base interface
    behavior_manager_->add(std::make_shared<tomm::MoveBase>("move_base"), false);

    // move to cartesian interface
    behavior_manager_->add(std::make_shared<tomm::MoveToCartesianMobileManipulator>("move_to_cartesian_mobile_manipulator"), true);

    if(!behavior_manager_->initRequest(nh, params_))
    {
      PRINT_ERROR("Failed to initialize whole body controller");
      return false;
    }

    // final checks
    if(controller_->formulation().idyn().robot().model().nq != comm_interface_.robotState().joints().q().size())
    {
      PRINT_ERROR("Number of joints in model %d and hardware %d dont match ", 
        int(controller_->formulation().idyn().robot().model().nq), 
        int(comm_interface_.robotState().joints().q().size()));
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Setup connecitons
    ////////////////////////////////////////////////////////////////////////////
    controller_start_pub_ = nh.advertise<std_msgs::Empty>("controller_start", 1);
    
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_state", 1);
    joint_state_msg_.name = controller_->cmdRobot().actuatedJointNames();

    return true;
  }

  void App::start(const ros::Time &time)
  {
    time_start_ = time;

    hardware_interface_->startRequest(time);
    cc::JointPosition start_posture = hardware_interface_->jointState().pos();
    cc::JointPosition home_posture = params_.home_posture;
    // home_posture.head(3) = start_posture.head(3); // TODO: This is wrong if no mobile base!
    
    PRINT_WARN_STREAM("q_init=" << start_posture.toString());
    PRINT_WARN_STREAM("home_posture=" << home_posture.toString());

    spline_ = std::make_unique<Trajectory>(params_.homing_period,
      start_posture, home_posture);
    PRINT_INFO("starting");
  }

  bool App::update(const ros::Time &time, const ros::Duration &period)
  {
    cc::Scalar elapsed = (time - time_start_).toSec();

    // update robot state
    if(!hardware_interface_->updateRequest(time, period))
    {
      PRINT_ERROR("Safty shutdown due to sensor error");
      return false;
    }

    // inital joint spline for homing motion
    if(elapsed < spline_->endTime())
    {
      if(!hardware_interface_->sendCommand(spline_->evaluate(elapsed)))
      {
        PRINT_ERROR("Safty shutdown due to invalid command");
        return false;
      }
    }
    else
    {
      // start if not running
      if(!behavior_manager_->isRunning())
      {
        PRINT_ERROR("Spline done, starting controller");
        behavior_manager_->startRequest(time);
        controller_start_pub_.publish(std_msgs::Empty());
      }

      // update controller
      if(behavior_manager_->updateRequest(time, period))
      {
        // send to the robot
        if(!hardware_interface_->sendCommand(behavior_manager_->command()))
        {
          PRINT_ERROR("Safty shutdown due to invalid command");
          return false;
        }

        // send jointstate
        joint_state_msg_ = behavior_manager_->command();
        joint_state_msg_.header.stamp = time;
        joint_state_pub_.publish(joint_state_msg_);
      }
      else
      {
        PRINT_ERROR("Safty shutdown due to controller error!");
        return false;
      }
    }

    return true;
  }

  void App::stop(const ros::Time &time)
  {
    behavior_manager_->stopRequest(time);
    PRINT_WARN("stopped");
  }

}