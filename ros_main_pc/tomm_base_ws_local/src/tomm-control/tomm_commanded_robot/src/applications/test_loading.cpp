#include <tomm_commanded_robot/commanded_robot.h>

#include <ics_tsid_tasks/tasks.h>

/**
 * @brief Test loading of commanded robot from tomm namespace
 * 
 */
int main(int argc, char **argv)
{
  ros::init(argc,argv,"commanded_robot", ros::init_options::AnonymousName);
  ros::NodeHandle nh("tomm");

  //////////////////////////////////////////////////////////////////////////////
  // register all tasks in task factory
  //////////////////////////////////////////////////////////////////////////////
  ics::register_task_types();

  //////////////////////////////////////////////////////////////////////////////
  // setup minimal global parameters
  //////////////////////////////////////////////////////////////////////////////
  cc::Parameters global_params;
  global_params.addOptional<cc::Scalar>("loop_rate", 200.0);
  global_params.addOptional<std::string>("prefix", "tomm");
  cc::JointPosition q_home = cc::JointPosition::Zero(15);
  q_home << 
       0, 0, 0,
      -2.32108, -1.14193, 1.33528, -1.90397, 0.779684, 0.710829,
       2.35406, -2.00985, -1.37966, -1.15071, -0.785414, -0.742987;
  global_params.addOptional<cc::JointPosition>("home_posture", q_home);
  global_params.load(nh);

  //////////////////////////////////////////////////////////////////////////////
  // load the commanded robot
  //////////////////////////////////////////////////////////////////////////////

  // load parameters, setup modules and states, projects robot on ground
  tomm::CommandedRobot commanded_robot("tomm");
  if(!commanded_robot.initRequest(nh, global_params))
  {
    ROS_ERROR("Error init commanded robot");
    return -1;
  }

  auto v_limit = commanded_robot.formulation().idyn().robot().model().velocityLimit;
  auto q_lo = commanded_robot.formulation().idyn().robot().model().lowerPositionLimit;
  auto q_up = commanded_robot.formulation().idyn().robot().model().upperPositionLimit;

  ROS_INFO_STREAM("----------------------------------------------------------");
  ROS_INFO_STREAM("v_limit=" << v_limit.transpose());
  ROS_INFO_STREAM("q_lo=" << q_lo.transpose());
  ROS_INFO_STREAM("q_up=" << q_up.transpose());
  ROS_INFO_STREAM("----------------------------------------------------------");

  //////////////////////////////////////////////////////////////////////////////
  // start commanded robot
  //////////////////////////////////////////////////////////////////////////////

  // updates joint and fb state, sets all tasks (normaly using real state)  
  commanded_robot.start(commanded_robot.jointState(), ros::Time::now());

  //////////////////////////////////////////////////////////////////////////////
  // check some of the states
  //////////////////////////////////////////////////////////////////////////////
  ROS_WARN_STREAM("Checking cartesian body states\n");

  ROS_INFO_STREAM("baseState=\n" 
    << commanded_robot.baseState().toString());

  ROS_INFO_STREAM("left_hand_state=\n"
    << commanded_robot.bodyState(cc::BodyId::LEFT_HAND).toString());

  ROS_INFO_STREAM("right_hand_state=\n" 
    << commanded_robot.bodyState(cc::BodyId::RIGHT_HAND).toString());

  //////////////////////////////////////////////////////////////////////////////
  // check if the solver is able to find a solution
  //////////////////////////////////////////////////////////////////////////////
  cc::JointState joint_state = commanded_robot.jointState();
  ros::Time time = ros::Time::now();
  ros::Duration period(1./200.);

  ROS_WARN_STREAM("Checking the solver\n");
  if(!commanded_robot.update(joint_state, time, period))
  {
    ROS_ERROR_STREAM("solver failed!");
  }
  else
  {
    ROS_WARN_STREAM("solution=\n " << commanded_robot.a().transpose());
  }

  return 0;
}