#include <tomm_skin/skin_visualizer.h>

SkinVisualizer::SkinVisualizer(
    const std::string &side, double prox_weight)
    : side_{side},
      prox_weight_{prox_weight}
{
  skin_patches_sub_ = nh_.subscribe(side_ + "_skin_patches", 1, &SkinVisualizer::skinCallback, this);

  uArm_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(side_ + "_upper_wrench", 1);
  lArm_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(side_ + "_lower_wrench", 1);
  hand_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(side_ + "_hand_wrench", 1);
}

SkinVisualizer::~SkinVisualizer()
{
}

void SkinVisualizer::broadcastCop()
{
  std::string ns = nh_.getNamespace() + "/";
  tf::StampedTransform lower_cop, upper_cop;
  lower_cop.frame_id_ = ns + lower_patch_.frame();
  lower_cop.child_frame_id_ = ns + side_ + "_lower_cop";
  lower_cop.stamp_ = ros::Time::now();
  lower_cop.setData(lower_patch_.pose());
  br_.sendTransform(lower_cop);

  upper_cop.frame_id_ = ns + upper_patch_.frame();
  upper_cop.child_frame_id_ = ns + side_ + "_upper_cop";
  upper_cop.stamp_ = ros::Time::now();
  upper_cop.setData(upper_patch_.pose());
  br_.sendTransform(upper_cop);

  // tf::StampedTransform hand_frame;
  // // cc::HomogeneousTransformation T_h_ee{hand_patch_.pose()};
  // hand_frame.frame_id_ = "hand_link_" + side_;
  // hand_frame.child_frame_id_ = side_ + "_hand";
  // hand_frame.stamp_ = ros::Time::now();
  // hand_frame.setData(hand_patch_.pose());
  // br_.sendTransform(hand_frame);
}

void SkinVisualizer::publishWrench()
{
  std::string ns = nh_.getNamespace() + "/";
  // remove the first '/' (frame id cannot start with '/')
  if (ns[0] == '/')
    ns = ns.substr(1);

  geometry_msgs::WrenchStamped lower_wrench, upper_wrench, hand_wrench;
  lower_wrench.header.frame_id = ns + side_ + "_lower_cop";
  lower_wrench.header.stamp = ros::Time::now();
  cc::Wrench W_lower = prox_weight_ * lower_patch_.proximity().wrench() +
                       (1 - prox_weight_) * lower_patch_.force().wrench();
  lower_wrench.wrench = W_lower;
  lArm_wrench_pub_.publish(lower_wrench);

  upper_wrench.header.frame_id = ns + side_ + "_upper_cop";
  upper_wrench.header.stamp = ros::Time::now();
  cc::Wrench W_upper = prox_weight_ * upper_patch_.proximity().wrench() +
                       (1 - prox_weight_) * upper_patch_.force().wrench();
  upper_wrench.wrench = W_upper;
  uArm_wrench_pub_.publish(upper_wrench);

  hand_wrench.header.frame_id = ns + "hand_link_" + side_;
  hand_wrench.header.stamp = ros::Time::now();
  cc::Wrench W_hand = prox_weight_ * hand_patch_.proximity().wrench() +
                      (1 - prox_weight_) * hand_patch_.force().wrench();
  hand_wrench.wrench = W_hand;
  hand_wrench_pub_.publish(hand_wrench);
}

void SkinVisualizer::skinCallback(const control_core_msgs::SkinPatches::ConstPtr &msg)
{
  lower_patch_ = msg->patches[0];
  upper_patch_ = msg->patches[1];
  hand_patch_ = msg->patches[2];
}