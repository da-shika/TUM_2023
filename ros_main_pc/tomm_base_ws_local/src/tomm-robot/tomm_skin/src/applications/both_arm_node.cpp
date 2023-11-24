#include <tomm_skin/skin_interface.h>
#include <tomm_skin/skin_visualizer.h>

#include <control_core_msgs/SkinPatches.h>
#include <geometry_msgs/WrenchStamped.h>

class SkinCombine
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber left_patches_sub_;
  ros::Subscriber right_patches_sub_;
  ros::Publisher arm_patches_pub_;
  ros::Publisher hand_patches_pub_;

  cc::SkinPatch left_lower_patch_, left_upper_patch_, left_hand_patch_;
  cc::SkinPatch right_lower_patch_, right_upper_patch_, right_hand_patch_;

  void leftskinCallback(const control_core_msgs::SkinPatches::ConstPtr &msg)
  {
    left_lower_patch_ = msg->patches[0];
    left_upper_patch_ = msg->patches[1];
    left_hand_patch_ = msg->patches[2];
  }

  void rightskinCallback(const control_core_msgs::SkinPatches::ConstPtr &msg)
  {
    right_lower_patch_ = msg->patches[0];
    right_upper_patch_ = msg->patches[1];
    right_hand_patch_ = msg->patches[2];
  }

public:
  SkinCombine()
  {
    left_patches_sub_ = nh_.subscribe("left_skin_patches", 1, &SkinCombine::leftskinCallback, this);
    right_patches_sub_ = nh_.subscribe("right_skin_patches", 1, &SkinCombine::rightskinCallback, this);
    arm_patches_pub_ = nh_.advertise<control_core_msgs::SkinPatches>("arm_patches", 1);
    hand_patches_pub_ = nh_.advertise<control_core_msgs::SkinPatches>("hand_patches", 1);
  }
  ~SkinCombine()
  {
  }

  void publishAllPatches()
  {
    control_core_msgs::SkinPatches arm_patches_msg;
    arm_patches_msg.patches.push_back(left_lower_patch_);
    arm_patches_msg.patches.push_back(left_upper_patch_);
    arm_patches_msg.patches.push_back(right_lower_patch_);
    arm_patches_msg.patches.push_back(right_upper_patch_);
    arm_patches_pub_.publish(arm_patches_msg);

    control_core_msgs::SkinPatches hand_patches_msg;
    hand_patches_msg.patches.push_back(left_hand_patch_);
    hand_patches_msg.patches.push_back(right_hand_patch_);
    hand_patches_pub_.publish(hand_patches_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_skin_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  std::string left_side = "left";
  std::string right_side = "right";

  tomm_skin::TommSkin l_skin(left_side);
  SkinVisualizer l_skin_vis(left_side);

  tomm_skin::TommSkin r_skin(right_side);
  SkinVisualizer r_skin_vis(right_side);
  SkinCombine arm_patches;

  if (!l_skin.config())
  {
    ROS_ERROR_STREAM("Failed to initialize left arm skin patches");
    return -1;
  }
  if (!r_skin.config())
  {
    ROS_ERROR_STREAM("Failed to initialize right arm skin patches");
    return -1;
  }
  l_skin.updateAll();
  r_skin.updateAll();

  // start async spinner
  ros::Rate rate(100);

  while (ros::ok())
  {
    l_skin.updateAll();
    l_skin_vis.broadcastCop();
    l_skin_vis.publishWrench();

    r_skin.updateAll();
    r_skin_vis.broadcastCop();
    r_skin_vis.publishWrench();

    arm_patches.publishAllPatches();

    rate.sleep();
    // ros::spinOnce();
  }
  // stop everything
  spinner.stop();
  ROS_WARN_STREAM("test_skin_node::main(): stopped");

  ros::waitForShutdown();

  return 0;
}