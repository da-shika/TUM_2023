#include <ros/ros.h>
#include <ros/package.h>
#include <controller_manager/controller_manager.h>

#include <tomm_hardware_real/tomm_hardware_real.h>

/**
 * @brief This is the ros control main update loop for whole body hardware
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
  // initialize ROS
  ros::init(argc, argv, "tomm_hardware_real_node", ros::init_options::NoSigintHandler);
  ROS_INFO("tomm_hardware_real_node::main(): inialized");

  // node handles
  ros::NodeHandle robot_nh;

  // path to arm config files
  std::string pkg_path = ros::package::getPath("tomm_hardware_real");
  std::string left_arm_config_file = pkg_path + "/configs/arm/left_arm_config.yaml";
  std::string right_arm_config_file = pkg_path + "/configs/arm/right_arm_config.yaml";
  std::string internal_pid_file = pkg_path + "/configs/arm/internal_pid.yaml";
  std::string base_config_file = pkg_path + "/configs/omnibase/omnibase_driver.yaml";

  std::string base_name = "omnibase_driver";

  tomm_hw::TOMMHardwareReal tomm(left_arm_config_file, right_arm_config_file, internal_pid_file, base_config_file, false);
  if (!tomm.init(robot_nh, robot_nh))
  {
    ROS_ERROR("tomm_hardware_real_node::main(): Failed to init TOMMHardwareReal");
    return -1;
  }

  // register object and node handle with the controller manager
  controller_manager::ControllerManager manager(&tomm, robot_nh);

  // start async spinner
  ros::Rate rate(500);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // start the arm thread
  ros::Time time = ros::Time::now();
  ros::Time start_time = time;
  ros::Duration dt(0);
  if (!tomm.start(start_time))
  {
    ROS_ERROR("tomm_hardware_arm_node::main(): Failed to get valid robot connection");
    return -1;
  }

  // start the main ros-control loop
  ROS_WARN_STREAM("tomm_hardware_arm_node::main(): Valid robot connection, starting");
  ros::Time prev_time = ros::Time::now();

  while (ros::ok() && tomm.hasValidConnection() && !tomm.shutDown())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Hardware Interface Updating");
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;

    tomm.read(time, dt);
    manager.update(time, dt);
    tomm.write(time, dt);

    ros::spinOnce();
    rate.sleep();
  }

  // write Zeros to the comm interface that are still alive
  time = ros::Time::now();
  dt = time - prev_time;
  tomm.writeZero(time, dt);

  // if tomm not shut down by H1, request H1 to shutdown
  if (!tomm.shutDown())
    tomm.requestH1ShutDown();

  // stop everything
  spinner.stop();
  ROS_WARN_STREAM("tomm_hardware_arm_node::main(): stopped");

  return 0;
}
