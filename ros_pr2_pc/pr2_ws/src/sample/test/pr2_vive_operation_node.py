#!/usr/bin/env python
#-*- coding: utf-8 -*-

"""
Track wrist_pos and wrist_rot
"""

import sys
import numpy as np

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, String, Float32MultiArray


def trajectory(target_position, target_rotation):
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # set base frame
    target_pose.pose.position.x = target_position[0]
    target_pose.pose.position.y = target_position[1]
    target_pose.pose.position.z = target_position[2]
    target_pose.pose.orientation.x = target_rotation[0]
    target_pose.pose.orientation.y = target_rotation[1]
    target_pose.pose.orientation.z = target_rotation[2]
    target_pose.pose.orientation.w = target_rotation[3]
    return target_pose


class PR2ViveControll:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    
        self.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.wait_for_service("/compute_ik")

        self.robot_state = self.robot.get_current_state()
        self.t_start = rospy.Time.now()

        self.config()

        self.cmd_sub = rospy.Subscriber("/robot_IO_cmd", String, self.cmdcallback)
        self.left_target_pos_sub = rospy.Subscriber("/pr2_left_target_pos", Float32MultiArray, self.leftposcallback)
        self.right_target_pos_sub = rospy.Subscriber("/pr2_right_target_pos", Float32MultiArray, self.rightposcallback)
        self.left_target_rot_sub = rospy.Subscriber("/pr2_left_target_rot", Float32MultiArray, self.leftrotcallback)
        self.right_target_rot_sub = rospy.Subscriber("/pr2_right_target_rot", Float32MultiArray, self.rightrotcallback)
        self.left_joint_pub = rospy.Publisher("/l_arm_controller/command", JointTrajectory, queue_size=10)
        self.right_joint_pub = rospy.Publisher("/r_arm_controller/command", JointTrajectory, queue_size=10)


    def config(self):
        self.cmd = None
        
        self.left_target_position = None
        self.left_target_rotation = None
        self.left_arm_joint_names = None
        self.left_arm_joint_positions = None

        self.right_target_position = None
        self.right_target_rotation = None
        self.right_arm_joint_names = None
        self.righ_arm_joint_positions = None


    def create_joint_trajectory_msg(self, joint_names, joint_positions, duration=1./60.):
        # set JointTrajectoryPoint message
        joint_trajectory = JointTrajectory()
        joint_trajectory.header = Header()
        joint_trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions=joint_positions
        point.time_from_start = rospy.Duration(duration)
        joint_trajectory.points.append(point)    
        return joint_trajectory


    def computeIK(self, target_position, target_rotation, side):
        # trajectory
        target_pose = trajectory(target_position, target_rotation)
        # compute ik
        req = GetPositionIKRequest()
        req.ik_request.group_name = side + "_arm"
        req.ik_request.robot_state = self.robot_state
        req.ik_request.avoid_collisions = False
        req.ik_request.pose_stamped = target_pose
        return req


    def move_both_arms(self):
        if self.cmd == "ON":
            if self.left_target_position is not None and self.right_target_position is not None:
                left_req = self.computeIK(self.left_target_position, self.left_target_rotation, "left")
                left_res = self.compute_ik_srv(left_req)
                self.left_arm_joint_names = left_res.solution.joint_state.name[15:22]
                self.left_arm_joint_positions = left_res.solution.joint_state.position[15:22]

                right_req = self.computeIK(self.right_target_position, self.right_target_rotation, "right")
                right_res = self.compute_ik_srv(right_req)
                self.right_arm_joint_names = right_res.solution.joint_state.name[-15:-8]
                self.right_arm_joint_positions = right_res.solution.joint_state.position[-15:-8]

                self.pub_joint_trajectory(self.left_arm_joint_positions, self.right_arm_joint_positions)
        else:
            pass


    def pub_joint_trajectory(self, left_joint_positions, right_joint_positions):
        left_joint_trajectory_msg = self.create_joint_trajectory_msg(self.left_arm_joint_names, left_joint_positions)
        right_joint_trajectory_msg = self.create_joint_trajectory_msg(self.right_arm_joint_names, right_joint_positions)
        self.left_joint_pub.publish(left_joint_trajectory_msg)
        self.right_joint_pub.publish(right_joint_trajectory_msg)


    def cmdcallback(self, msg):
        self.cmd = msg.data

    def leftposcallback(self, msg):
        self.left_target_position = msg.data

    def leftrotcallback(self, msg):
        self.left_target_rotation = msg.data
    
    def rightrotcallback(self, msg):
        self.right_target_rotation = msg.data

    def rightposcallback(self, msg):
        self.right_target_position = msg.data


def main():
    arm = PR2ViveControll()

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        arm.move_both_arms()
        rate.sleep()

    # print(k)

if __name__ == '__main__':
    rospy.init_node("pr2_moveit_online")
    main()

