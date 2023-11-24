#!/usr/bin/env python3

"""
Track wrist_pos and wrist_rot
This script don't work in VR computer
"""

import sys
import numpy as np

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, String, Float32MultiArray, Float32
from sensor_msgs.msg import JointState

from test.hand_controller import PR2GripperController


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

        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.cmd_sub = rospy.Subscriber("/robot_IO_cmd", String, self.cmdcallback)
        self.initial_pos_cmd_sub = rospy.Subscriber("/robot_initial_pos_cmd", String, self.initial_cmd_callback)
        self.left_hand_cmd_sub = rospy.Subscriber("/pr2_left_gripper_cmd", Float32, self.left_hand_cmd_callback)
        self.right_hand_cmd_sub = rospy.Subscriber("/pr2_right_gripper_cmd", Float32, self.right_hand_cmd_callback)

        self.left_target_pos_sub = rospy.Subscriber("/pr2_left_target_pos", Float32MultiArray, self.leftposcallback)
        self.right_target_pos_sub = rospy.Subscriber("/pr2_right_target_pos", Float32MultiArray, self.rightposcallback)
        self.left_target_rot_sub = rospy.Subscriber("/pr2_left_target_rot", Float32MultiArray, self.leftrotcallback)
        self.right_target_rot_sub = rospy.Subscriber("/pr2_right_target_rot", Float32MultiArray, self.rightrotcallback)

        self.left_joint_pub = rospy.Publisher("/l_arm_controller/command", JointTrajectory, queue_size=10)
        self.right_joint_pub = rospy.Publisher("/r_arm_controller/command", JointTrajectory, queue_size=10)


    def config(self):
        self.hand_controller = PR2GripperController()

        self.cmd = None
        self.initial_pos_cmd = None
        self.left_hand_cmd = None
        self.right_hand_cmd = None
        self.is_robot_initial_pos = True

        self.left_initial_position = np.array([-0.0045037736244504245, -0.00011895716422749558, 0.006574954580981007, 7.165849240564626e-05, -0.306538116205914, -0.43759435803385127, 0.0003388245909397014])
        self.right_initial_position = np.array([0.0015158762067342124, -3.3892049154715664e-05, 0.0066137725178680995, 0.00024150218120677636, -0.3075269110404957, -0.4364114258855327, -0.00020473830900602508])
        
        self.left_target_position = None
        self.left_target_rotation = None
        self.left_arm_joint_names = None
        self.left_arm_joint_positions = None

        self.right_target_position = None
        self.right_target_rotation = None
        self.right_arm_joint_names = None
        self.righ_arm_joint_positions = None


    def joint_state_callback(self, msg):
        self.left_arm_joint_names = msg.name[-14:-7]
        self.right_arm_joint_names = msg.name[17:24]
    def cmdcallback(self, msg):
        self.cmd = msg.data
    def initial_cmd_callback(self, msg):
        self.initial_pos_cmd = msg.data
    def left_hand_cmd_callback(self, msg):
        self.left_hand_cmd = msg.data
    def right_hand_cmd_callback(self, msg):
        self.right_hand_cmd = msg.data
    def leftposcallback(self, msg):
        self.left_target_position = msg.data
    def leftrotcallback(self, msg):
        self.left_target_rotation = msg.data  
    def rightrotcallback(self, msg):
        self.right_target_rotation = msg.data
    def rightposcallback(self, msg):
        self.right_target_position = msg.data

#----------------------------------------------------------------------------------------------
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
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
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
                    self.is_robot_initial_pos = False
            elif self.cmd == "OFF":
                self.return_initial_pos()
            self.gripper_controll()
            rate.sleep()

    def return_initial_pos(self):
        if self.is_robot_initial_pos == True:
            pass
        elif self.is_robot_initial_pos == False:
            if self.initial_pos_cmd == "Move to initial pos":
                left_initial_pos_trajectory = self.create_joint_trajectory_msg(self.left_arm_joint_names, self.left_initial_position)
                right_initial_pos_trajectory = self.create_joint_trajectory_msg(self.right_arm_joint_names, self.right_initial_position)
                self.left_joint_pub.publish(left_initial_pos_trajectory)
                self.right_joint_pub.publish(right_initial_pos_trajectory)
                self.is_robot_initial_pos = True
                self.initial_pos_cmd = None

    def gripper_controll(self):
        if self.left_hand_cmd == 0:
            self.hand_controller.left_close()
        elif self.left_hand_cmd == 1:
            self.hand_controller.left_open()

        if self.right_hand_cmd == 0:
            self.hand_controller.right_close()
        elif self.right_hand_cmd == 1:
            self.hand_controller.right_open()
#-----------------------------------------------------------------------------------------------------
    def pub_joint_trajectory(self, left_joint_positions, right_joint_positions):
        left_joint_trajectory_msg = self.create_joint_trajectory_msg(self.left_arm_joint_names, left_joint_positions)
        right_joint_trajectory_msg = self.create_joint_trajectory_msg(self.right_arm_joint_names, right_joint_positions)
        self.left_joint_pub.publish(left_joint_trajectory_msg)
        self.right_joint_pub.publish(right_joint_trajectory_msg)


def main():
    arm = PR2ViveControll()
    arm.move_both_arms()
    # print(k)

if __name__ == '__main__':
    rospy.init_node("pr2_moveit_online")
    main()

