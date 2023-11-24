#include <ros/ros.h>
#include <control_core/types.h>

#include <control_core_msgs/SkinPatches.h>
#include <geometry_msgs/WrenchStamped.h>

class SkinVis
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber hand_patches_sub_;
  ros::Publisher left_wrench_pub_;
  ros::Publisher right_wrench_pub_;

  cc::SkinPatch left_hand_patch_;
  cc::SkinPatch right_hand_patch_;

  void skinCallback(const control_core_msgs::SkinPatches::ConstPtr &msg)
  {
    left_hand_patch_ = msg->patches[0];
    right_hand_patch_ = msg->patches[1];

    geometry_msgs::WrenchStamped left_msg;
    left_msg.header.frame_id = "hand_link_left";
    left_msg.header.stamp = ros::Time::now();
    left_msg.wrench = left_hand_patch_.force().wrench();
    left_wrench_pub_.publish(left_msg);

    geometry_msgs::WrenchStamped right_msg;
    right_msg.header.frame_id = "hand_link_right";
    right_msg.header.stamp = ros::Time::now();
    right_msg.wrench = right_hand_patch_.force().wrench();
    right_wrench_pub_.publish(right_msg);
    ROS_WARN_STREAM_THROTTLE(0.5, "published");
  }

public:
  SkinVis()
  {
    hand_patches_sub_ = nh_.subscribe("hand_patches", 1, &SkinVis::skinCallback, this);
    left_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("left_hand_wrench_new", 1);
    right_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("right_hand_wrench_new", 1);
  }

  ~SkinVis()
  {
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle nh;

  SkinVis skin_vis;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}