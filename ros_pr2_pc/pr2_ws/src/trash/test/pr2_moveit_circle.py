#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import numpy as np
import timeit

def trajectory(c=np.array([0.5, 0.0, 0.7]), f=0.1, r=0.1, t=0.0):
    x = c + r*np.array([0.0, np.cos(2*np.pi*f*t), np.sin(2*np.pi*f*t)])

    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # 基準フレームを指定
    target_pose.pose.position.x = x[0]
    target_pose.pose.position.y = x[1]
    target_pose.pose.position.z = x[2]
    target_pose.pose.orientation.w = 1.0
    
    return target_pose

class MovePR2Arm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("right_arm")  # PR2の右腕を制御する例
        self.t_start = None

    def home(self, time):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"  # 基準フレームを指定
        pose.pose.position.x = 0.5
        pose.pose.position.y = -0.1
        pose.pose.position.z = 0.7
        pose.pose.orientation.w = 1.0

        self.arm_group.set_pose_target(pose)
        self.arm_group.set_start_state_to_current_state()
        plan = self.arm_group.plan()
        self.arm_group.execute(plan)

        self.t_start = time

    def run(self, time):
        t = (time - self.t_start).to_sec()
        pose = trajectory(np.array([0.5, -0.1, 0.7]), 0.1, 0.1, t)
        
        self.arm_group.set_pose_target(pose)
        # self.arm_group.set_start_state_to_current_state()
        plan = self.arm_group.plan()
        self.arm_group.execute(plan)

if __name__ == '__main__':
    rospy.init_node("pr2_moveit_circle")
    
    move_arm = MovePR2Arm()

    move_arm.home(rospy.Time.now())

    while not rospy.is_shutdown():
        start = rospy.Time.now()
        move_arm.run(rospy.Time.now())
        end = rospy.Time.now()

        rospy.logwarn_throttle(1.0, (end - start).to_sec())

