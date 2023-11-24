#include <ros/ros.h>
#include <tomm_hardware_real/utilities/joint_state.h>

using namespace tomm_hw;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_joint_state");

  JointState<DOF_ARM> js_left;
  ROS_WARN_STREAM("left position: "
                  << js_left.q().transpose());
  ROS_WARN_STREAM("left velocity:"
                  << js_left.vel().transpose());

  js_left.pos() << 0, 0, M_PI_2, 0, 0, 0;

  JointState<DOF_ARM> js_right{js_left};
  ROS_WARN_STREAM("right position: "
                  << js_right.pos().transpose());

  JointState<DOF_BASE> js_base;
  ROS_WARN_STREAM("base acc: "
                  << js_base.acc().transpose());

  return 0;
}