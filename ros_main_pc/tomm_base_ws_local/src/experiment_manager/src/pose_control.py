#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose
import pinocchio as pin
import numpy as np
from enum import Enum
from behavior_msgs.srv import ChangeBehavior, ChangeBehaviorRequest, ListBehavior, ListBehaviorRequest


def is_teleop_running(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/list_behavior', ListBehavior)
    req = srv_client.call(ListBehaviorRequest())
    return name in req.running_behaviors

def disable_teleop(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/change_behavior', ChangeBehavior)
    req = ChangeBehaviorRequest()
    req.start_behaviors = []
    req.stop_behaviors = [name]
    srv_client.call(req)

def enable_teleop(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/change_behavior', ChangeBehavior)
    req = ChangeBehaviorRequest()
    req.start_behaviors = [name]
    req.stop_behaviors = []
    srv_client.call(req)

def linear_interpolation(q0, q1, T, t):
    """Linear interpolation between q0 and q1"""
    s = min(1.0, max(0.0, t / T))
    return (q1 - q0)*s + q0

#-----------------------------------------------------------------------------------------------------------------------------------------
class PoseControl:
    class Mode(Enum):
        NONE = 1
        LINEAR = 2

    def __init__(self, controll_freq, target_freq, max_target_jump, mode=Mode.NONE):
        rospy.sleep(1.0)

        self.controll_freq = controll_freq
        self.max_target_delay = 5*1./10.
        self.target_period = 1./ target_freq
        self.max_target_jump = max_target_jump
        self.mode = mode
        self.config()
        self.left_target_msg = PoseStamped()
        self.right_target_msg = PoseStamped()

        # target input (nn outputs)
        self.left_target_sub = rospy.Subscriber("/target/left_pose", PoseStamped, self.left_target_callback)
        self.right_target_sub = rospy.Subscriber("/target/right_pose", PoseStamped, self.right_target_callback)

        # current input (robot feedback)
        self.left_current_sub = rospy.Subscriber("/tomm/teleop_left_hand/pose", PoseStamped, self.left_current_callback)
        self.right_current_sub = rospy.Subscriber("/tomm/teleop_right_hand/pose", PoseStamped, self.right_current_callback)

        # outputs
        self.left_pose_pub = rospy.Publisher("/tomm/teleop/hand_link_left_pose", PoseStamped, queue_size=1)
        self.right_pose_pub = rospy.Publisher("/tomm/teleop/hand_link_right_pose", PoseStamped, queue_size=1)


    def config(self):
        self.has_targets = False
        self.left_target_msg, self.right_target_msg = PoseStamped(), PoseStamped()
        self.left_pub_msg, self.right_pub_msg = PoseStamped(), PoseStamped()

        self.left_current_pose, self.right_current_pose = None, None
        self.left_target_pose, self.right_target_pose = None, None
        self.left_pub_command, self.right_pub_command = None, None
        self.left_prev_target, self.right_prev_target = None, None
        self.target_update_time = None

    def update(self):

        if self.left_current_pose is not None and self.right_current_pose is not None:

            # make target position (chose interpolate or not)
            if not self.has_targets:
                rospy.logwarn_throttle(0.5, "OnlineExecutor.run(): Wait for first target")
                self.left_pub_command = self.left_current_pose
                self.right_pub_command = self.right_current_pose
                self.left_prev_target = np.copy(self.left_current_pose)
                self.left_prev_target = np.copy(self.right_current_pose)

            else:
                rospy.logwarn_throttle(0.5, "Valuable target")
                t = (rospy.Time.now() - self.target_update_time).to_sec()
                if self.mode == PoseControl.Mode.LINEAR:
                    pass
                    # self.left_pub_command = linear_interpolation(self.left_prev_target, self.left_target_pose, self.target_period, t)
                    # self.right_pub_command = linear_interpolation(self.right_prev_target, self.right_target_pose, self.target_period, t)
                elif self.mode == PoseControl.Mode.NONE:
                    self.left_pub_command = self.left_target_pose
                    self.right_pub_command = self.right_target_pose

                if t > self.max_target_delay:
                    self.has_targets = False
                    rospy.logwarn("OnlineExecutor.run(): Interpolation stopped, wait for valid command: {0}s > {1}s".format(t, self.max_target_delay))

            self.publish()

    def publish(self):    
        self.left_pub_msg.pose = self.left_pub_command.pose
        self.left_pub_msg.header.stamp = rospy.Time.now()
        self.left_pub_msg.header.frame_id = 'tomm/base_link'
        self.left_pose_pub.publish(self.left_pub_msg)

        self.right_pub_msg.pose = self.right_pub_command.pose
        self.right_pub_msg.header.stamp = rospy.Time.now()
        self.right_pub_msg.header.frame_id = 'tomm/base_link'
        self.right_pose_pub.publish(self.right_pub_msg)

#-----------------------------------------------------------------------------------------------------------------------------------------
    def left_target_callback(self, msg):
        if self.right_target_pose is not None:
            self.has_targets = True
        self.target_update_time = rospy.Time.now()
        self.left_target_pose = msg

    def right_target_callback(self, msg):
        if self.left_target_pose is not None:
            self.has_targets = True
        self.target_update_time = rospy.Time.now()
        self.right_target_pose = msg

    def left_current_callback(self, msg):
        self.left_current_pose = msg

    def right_current_callback(self, msg):
        self.right_current_pose = msg

#-----------------------------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node("pose_controller_node")

    #---------------------------------------------------------------------
    # turn on teleop
    rospy.loginfo('starting teleop')
    enable_teleop()
    rospy.sleep(2.0)

    if not is_teleop_running():
        rospy.logerr('error: could not turn on teleoperation')
        return

    #---------------------------------------------------------------------
    rospy.logwarn("pose_controller_node start")
    controller = PoseControl(controll_freq=60,
        target_freq=10,
        max_target_jump=100)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()

    #---------------------------------------------------------------------

    #disable_teleop()
    rospy.logwarn("pose_controller_node done")

    rospy.loginfo('starting teleop')
    disable_teleop()
    rospy.sleep(2.0)


if __name__ == "__main__":
    main()