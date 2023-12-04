#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class PR2ArmMover:
    def __init__(self):
        rospy.init_node('pr2_ik_solver')
        rospy.loginfo("pr2_ik_solver_node start")
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    def move_arm(self):
        # Set the goal pose
        left_target_pose = PoseStamped()
        left_target_pose.header.frame_id = "base_link"  # Set base frame
        left_target_pose.pose.position.x = 0.5
        left_target_pose.pose.position.y = 0.5
        left_target_pose.pose.position.z = 1.0
        left_target_pose.pose.orientation.w = 1.0

        # IK calculation
        self.left_arm_group.set_pose_target(left_target_pose)
        self.left_arm_group.set_start_state_to_current_state()
        left_plan = self.left_arm_group.plan()

        # Set the goal pose
        right_target_pose = PoseStamped()
        right_target_pose.header.frame_id = "base_link"  # Set base frame
        right_target_pose.pose.position.x = 0.5
        right_target_pose.pose.position.y = -0.5
        right_target_pose.pose.position.z = 1.0
        right_target_pose.pose.orientation.w = 1.0
        
        # IK calculation
        self.right_arm_group.set_pose_target(right_target_pose)
        self.right_arm_group.set_start_state_to_current_state()
        right_plan = self.right_arm_group.plan()

        # Execute the plan
        self.left_arm_group.execute(left_plan)
        self.right_arm_group.execute(right_plan)

if __name__ == '__main__':
    try:
        pr2_arm_mover = PR2ArmMover()
        pr2_arm_mover.move_arm()
    except rospy.ROSInterruptException:
        pass