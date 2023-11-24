#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

class GetCurrentState:
    def __init__(self):
        rospy.init_node("get_current_state_node")
        rospy.loginfo("get_current_state_node start")

        self.torso_joint = None
        self.torso_name = None
        self.head_joint = None
        self.head_name = None
        self.left_joint = None
        self.left_name = None
        self.right_joint = None
        self.right_name = None

        self.state_sub = rospy.Subscriber("/joint_states", JointState, self.state_callback)

    def state_callback(self, msg):
        self.torso_joint = msg.position[12]
        self.torso_name = msg.name[12]
        self.head_joint = msg.position[14:16]
        self.head_name = msg.name[14:16]
        self.left_joint = msg.position[-14:-7]
        self.left_name = msg.name[-14:-7]
        self.right_joint = msg.position[17:24]
        self.right_name = msg.name[17:24]

    def run(self):
        print("Torso:", self.torso_joint)
        print("Torso:", self.torso_name)
        print("Head:", self.head_joint)
        print("Head:", self.head_name)
        print("Left:", self.left_joint)
        print("Left:", self.left_name)
        print("Right:", self.right_joint)
        print("Right:", self.right_name)

if __name__ == "__main__":
    get_current_state = GetCurrentState()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        get_current_state.run()
        rate.sleep()