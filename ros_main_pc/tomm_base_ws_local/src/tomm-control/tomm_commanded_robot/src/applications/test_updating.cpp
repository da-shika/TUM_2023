#include <tomm_commanded_robot/commanded_robot.h>

#include <ics_tsid_tasks/tasks.h>

/**
 * @brief Test updating of commanded robot from tomm namespace
 * 
 */
int main(int argc, char **argv)
{
  ros::init(argc,argv,"commanded_robot", ros::init_options::AnonymousName);
  ros::NodeHandle nh("tomm");

  ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  tf::TransformBroadcaster broadcaster;

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
  //////////////////////////////////////////////////////////////////////////////
  // start commanded robot
  //////////////////////////////////////////////////////////////////////////////

  // updates joint and fb state, sets all tasks (normaly using real state)  
  commanded_robot.start(commanded_robot.jointState(), ros::Time::now());

  //////////////////////////////////////////////////////////////////////////////
  // run commanded robot
  //////////////////////////////////////////////////////////////////////////////

  auto joint_names = commanded_robot.formulation().idyn().robot().actuatedJointNames();
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.position.resize(joint_names.size());

  tf::StampedTransform odometry;
  odometry.frame_id_ = "world";
  odometry.child_frame_id_ = "base_link";                                     

  ros::Rate rate(200.0);
  ros::Time time, prev_time;
  ros::Duration period;
  prev_time = ros::Time::now();
  while(ros::ok())
  {
    time = ros::Time::now();
    period = time - prev_time;
    prev_time = time;

    // use own state as next input
    const auto& joint_state = commanded_robot.jointState();

    // solve and integrate
    commanded_robot.update(joint_state, time, period);

    ROS_INFO_STREAM_THROTTLE(0.5, "q=" << commanded_robot.jointState().pos().toString());

    // send jointstate
    for(size_t i = 0; i < joint_names.size(); ++i)
      joint_state_msg.position[i] = commanded_robot.jointState().pos()[i+3];
    joint_state_msg.header.stamp = time;
    joint_state_msg.name = joint_names;
    joint_states_pub.publish(joint_state_msg);

    // send fb transformation
    odometry.setData(commanded_robot.baseState().pos().toTransformTf());
    odometry.stamp_ = time;
    broadcaster.sendTransform(odometry);  

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}