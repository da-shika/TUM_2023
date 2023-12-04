#! /usr/bin/env python

import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState

class CurrentStateCheck:
    def __init__(self):
        rospy.init_node("current_state_check_node")
        rospy.loginfo("current_state_check_node start")

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        robot_state = self.robot.get_current_state()

        self.left_joint = robot_state.joint_state.position[15:22]
        self.left_name = robot_state.joint_state.name[15:22]
        self.right_joint = robot_state.joint_state.position[-15:-8]
        self.right_name = robot_state.joint_state.name[-15:-8]

    def check(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("L:", self.left_joint)
            print("L:", self.left_name)
            print("R:", self.right_joint)
            print("R:", self.right_name)
            rate.sleep()

if __name__ == "__main__":
    currentstatecheck = CurrentStateCheck()
    currentstatecheck.check()