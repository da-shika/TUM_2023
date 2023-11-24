#include <ros/ros.h>
#include <ros/package.h>
#include <tomm_hardware_real/omnibase/omnibase_interface.h>

/**
 * @brief Test loading the configuration files
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_base_interface_node");

  ros::NodeHandle root_nh;
  ros::NodeHandle hw_nh;

  std::string pkg_path = ros::package::getPath("tomm_hardware_real");
  std::string config_file = pkg_path + "/configs/omnibase/omnibase_driver.yaml";

  // setup left arm
  tomm_hw::OmnibaseInterface omnibase_comm_interface(config_file);
  if (!omnibase_comm_interface.init(root_nh, hw_nh))
  {
    ROS_ERROR_STREAM("Failed to initalize OmnibaseInterface");
    return -1;
  }

  tomm_hw::JointState<DOF_BASE> js_base;
  tomm_hw::JointState<DOF_BASE> js_cmd;
  js_cmd.setZero();

  // start internal thread
  omnibase_comm_interface.start();

  // read data
  ros::Rate rate(500);
  ros::Time time;
  ros::Time prev_time;
  ros::Time cmd_time;
  ros::Duration dt;

  ros::Time start_time;
  bool is_started = false;

  while (ros::ok())
  {
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;

    if (!is_started && omnibase_comm_interface.hasValidState())
    {
      is_started = true;
      start_time = ros::Time::now();
    }

    if (is_started)
    {
      if (omnibase_comm_interface.read(js_base, time, dt))
      {
        ROS_WARN_STREAM_THROTTLE(0.5, "js_base_pos=" << js_base.pos().transpose());
        ROS_WARN_STREAM_THROTTLE(0.5, "js_base_vel=" << js_base.vel().transpose() << "\n");
      }
      
      // move forward with 0.05m/s for 5s then stop
      if (time - start_time < ros::Duration(5))
      {
        js_cmd.vel() << 0.05, 0, 0;
        if (omnibase_comm_interface.write(js_cmd, time, dt))
        {
          ROS_INFO_STREAM_THROTTLE(0.5, "cmd_vel=" << js_cmd.vel().transpose() << "\n");
        }
      }
      else
      {
        js_cmd.vel() << 0, 0, 0;
        if (omnibase_comm_interface.write(js_cmd, time, dt))
        {
          ROS_INFO_STREAM_THROTTLE(0.5, "cmd_vel=" << js_cmd.vel().transpose() << "\n");
        }
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  // shutdown
  omnibase_comm_interface.stop();
  return 0;
}