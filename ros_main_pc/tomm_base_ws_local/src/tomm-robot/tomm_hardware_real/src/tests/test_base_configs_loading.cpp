#include <ros/ros.h>
#include <ros/package.h>
#include <tomm_hardware_real/omnibase/omnibase_configs.h>

/**
 * @brief Test loading the configuration files
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_base_configs_loading");
  ros::NodeHandle nh;

  // path to arm config files
  std::string pkg_path = ros::package::getPath("tomm_hardware_real");
  std::string config_file = pkg_path + "/configs/omnibase/omnibase_driver.yaml";

  tomm_hw::OmnibaseConfig config_base;
  if(!config_base.load(config_file))
  {
    ROS_ERROR_STREAM("Failed to load config!");
    return -1;
  }
  ROS_WARN_STREAM("OmnibaseConfig:\n" << config_base.toString());

  return 0;
}