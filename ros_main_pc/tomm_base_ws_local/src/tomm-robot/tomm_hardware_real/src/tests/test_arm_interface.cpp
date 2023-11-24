#include <ros/ros.h>
#include <ros/package.h>
#include <tomm_hardware_real/arm/arm_interface.h>

/**
 * @brief Test loading the configuration files
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_arm_comm_interface_node");

  ros::NodeHandle root_nh;
  ros::NodeHandle hw_nh;

  // path to arm config files
  std::string pkg_path = ros::package::getPath("tomm_hardware_real");
  std::string left_arm_config_file = pkg_path + "/configs/arm/left_arm_config.yaml";
  std::string right_arm_config_file = pkg_path + "/configs/arm/right_arm_config.yaml";
  std::string internal_pid_config_file = pkg_path + "/configs/arm/internal_pid.yaml";

  // setup left arm
  tomm_hw::ArmInterface left_arm_comm_interface(left_arm_config_file, internal_pid_config_file);
  if (!left_arm_comm_interface.init(root_nh, hw_nh))
  {
    ROS_ERROR_STREAM("Failed to initalize ArmInterface");
    return -1;
  }

  // setup right arm
  tomm_hw::ArmInterface right_arm_comm_interface(right_arm_config_file, internal_pid_config_file);
  if (!right_arm_comm_interface.init(root_nh, hw_nh))
  {
    ROS_ERROR_STREAM("Failed to initalize ArmInterface");
    return -1;
  }

  tomm_hw::JointState<DOF_ARM> js_left, js_right;

  // start internal thread
  left_arm_comm_interface.start();
  right_arm_comm_interface.start();

  // read data
  ros::Rate rate(250);
  ros::Time time;
  ros::Time prev_time;
  ros::Duration dt;

  while (ros::ok())
  {
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;

    if (left_arm_comm_interface.read(js_left, time, dt))
    {
      ROS_WARN_STREAM_THROTTLE(0.5, "js_left=" << js_left.pos().transpose());
    }
    if (right_arm_comm_interface.read(js_right, time, dt))
    {
      ROS_WARN_STREAM_THROTTLE(0.5, "js_right=" << js_right.pos().transpose() << "\n");
    }

    ros::spinOnce();
    rate.sleep();
  }

  // shutdown
  left_arm_comm_interface.stop();
  right_arm_comm_interface.stop();
  return 0;
}