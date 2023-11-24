#include <ros/ros.h>
#include <ros/package.h>  
#include <tomm_hardware_real/arm/arm_configs.h>

/**
 * @brief Test loading the configuration files
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_arm_yaml_configs_loading");

  // path to arm config files
  std::string pkg_path = ros::package::getPath("tomm_hardware_real");
  std::string left_arm_config_file = pkg_path + "/configs/arm/left_arm_config.yaml";
  std::string right_arm_config_file = pkg_path + "/configs/arm/right_arm_config.yaml";

  tomm_hw::ArmConfig config_left;
  if(!config_left.load(left_arm_config_file))
  {
    ROS_ERROR_STREAM("Failed to load left config!");
  }
  else
    ROS_WARN_STREAM("ArmConfig LEFT:\n" << config_left.toString());

  tomm_hw::ArmConfig config_right;
  if(!config_right.load(right_arm_config_file))
  {
    ROS_ERROR_STREAM("Failed to load right config!");
  }
  else
    ROS_WARN_STREAM("ArmConfig RIGHT:\n" << config_right.toString());

  return 0;
}