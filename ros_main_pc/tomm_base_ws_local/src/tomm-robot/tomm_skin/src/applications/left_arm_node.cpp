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

  std::string side = "left";

  tomm_skin::TommSkin l_skin(side);
  SkinVisualizer l_skin_vis(side, 1.0);

  if (!l_skin.config())
  {
    ROS_ERROR_STREAM("Failed to initialize left arm skin patches");
    return -1;
  }

  l_skin.updateAll();

  // start async spinner
  ros::Rate rate(100);

  while (ros::ok())
  {
    l_skin.updateAll();
    l_skin_vis.broadcastCop();
    l_skin_vis.publishWrench();

    rate.sleep();
  }
  // stop everything
  spinner.stop();
  ROS_WARN_STREAM("test_skin_node::main(): stopped");

  ros::waitForShutdown();

  return 0;
}