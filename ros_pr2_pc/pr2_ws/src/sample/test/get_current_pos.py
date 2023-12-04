#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class PositionListener:
    def __init__(self):
        rospy.init_node("pr2_position_listener")
        rospy.loginfo("pr2_position_listener_node start")
        self.config()

        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_callback)

    def config(self):
        self.left_joint_names = None
        self.left_joint_position = None
        self.right_joint_names = None
        self.right_joint_position = None

    def joint_callback(self, msg):
        self.left_joint_names = msg.name[-14:-7]
        self.left_joint_position = msg.position[-14:-7]
        self.right_joint_names = msg.name[17:24]
        self.right_joint_position = msg.position[17:24]

    def display(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.left_joint_names is not None and self.left_joint_position is not None and self.right_joint_names is not None and self.right_joint_position is not None:
                print("Left joint name:", self.left_joint_names)
                print("Left joint position:", self.left_joint_position)
                print("Right joint name:", self.right_joint_names)
                print("Right joint name:", self.right_joint_position)
            else:
                rospy.loginfo("Wait valid value")
            rate.sleep()

if __name__ == "__main__":
    position_listener = PositionListener()
    position_listener.display()