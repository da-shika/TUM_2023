#include <tomm_skin/skin_interface.h>
#include <tomm_skin/skin_visualizer.h>

#include <control_core_msgs/SkinPatches.h>
#include <geometry_msgs/WrenchStamped.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_skin_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  std::string side = "right";

  tomm_skin::TommSkin r_skin(side);
  SkinVisualizer r_skin_vis(side);

  if (!r_skin.config())
  {
    ROS_ERROR_STREAM("Failed to initialize right arm skin patches");
    return -1;
  }
  r_skin.updateAll();

  // start async spinner
  ros::Rate rate(100);

  while (ros::ok())
  {
    r_skin.updateAll();
    r_skin_vis.broadcastCop();
    r_skin_vis.publishWrench();

    rate.sleep();
  }
  
  // stop everything
  spinner.stop();
  ROS_WARN_STREAM("test_skin_node::main(): stopped");

  ros::waitForShutdown();

  return 0;
}