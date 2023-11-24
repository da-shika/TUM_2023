#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <control_core_msgs/SkinPatches.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_core/types.h>

class SkinVisualizer
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber skin_patches_sub_;
  ros::Publisher uArm_wrench_pub_;
  ros::Publisher lArm_wrench_pub_;
  ros::Publisher hand_wrench_pub_;

  tf::TransformBroadcaster br_;

  std::string side_;

  double prox_weight_;

  cc::SkinPatch lower_patch_, upper_patch_, hand_patch_;

  void skinCallback(const control_core_msgs::SkinPatches::ConstPtr &msg);

public:
  SkinVisualizer(const std::string &side, double prox_weight = 0.3);
  ~SkinVisualizer();

  void broadcastCop();

  void publishWrench();
};