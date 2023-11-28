#!/usr/bin/env python
#-*- coding: utf-8 -*-

"""
Move by JointTrajectory
"""

import sys
import numpy as np

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, String

def trajectory(c=np.array([0.5, 0.0, 0.7]), f=0.1, r=0.1, t=0.0):
    x = c + r * np.array([0.0, np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t)])
    
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # set base frame
    target_pose.pose.position.x = x[0]
    target_pose.pose.position.y = x[1]
    target_pose.pose.position.z = x[2]
    target_pose.pose.orientation.w = 1.0
    return target_pose


class MovePR2Arm:
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

        self.initial_cmd_sub = rospy.Subscriber("/initial_cmd", String, self.initial_callback)
        self.left_joint_pub = rospy.Publisher("/l_arm_controller/command", JointTrajectory, queue_size=10)
        self.right_joint_pub = rospy.Publisher("/r_arm_controller/command", JointTrajectory, queue_size=10)

    def config(self):
        self.left_arm_joint_names = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint',
                                        'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        self.left_arm_joint_positions = None
        self.right_arm_joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint',
                                        'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
        self.righ_arm_joint_positions = None
        self.is_initial_position = False
        self.is_first_loop = True
        self.initial_cmd = None

    def create_joint_trajectory_msg(self, joint_names, joint_positions, duration=1./60.):
        # set JointTrajectoryPoint message
        joint_trajectory = JointTrajectory()
        joint_trajectory.header = Header()
        joint_trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(duration)
        joint_trajectory.points.append(point)    
        return joint_trajectory

    def initial_callback(self, msg):
        self.initial_cmd = msg.data
#-----------------------------------------------------------------------------------------------------------------
    def computeIK(self, time, c, side):
        # trajectory
        t = (time - self.t_start).to_sec()
        target_pose = trajectory(c, 0.5, 0.1, t)
        # compute ik
        req = GetPositionIKRequest()
        req.ik_request.group_name = side + "_arm"
        req.ik_request.robot_state = self.robot_state
        req.ik_request.avoid_collisions = False
        req.ik_request.pose_stamped = target_pose
        return req

    def move_both_arms(self, time):
        if self.is_first_loop == True:
            self.initial_position()
            self.is_first_loop = False
        
        if self.initial_cmd == "i":
            self.initial_position()

        elif self.initial_cmd == "m":
            self.is_initial_position = False
            if self.is_initial_position == False:
                left_req = self.computeIK(time, [0.5, 0.1, 0.7], "left")
                left_res = self.compute_ik_srv(left_req)
                self.left_arm_joint_names = left_res.solution.joint_state.name[15:22]
                self.left_arm_joint_positions = left_res.solution.joint_state.position[15:22]

                right_req = self.computeIK(time, [0.5, -0.1, 0.7], "right")
                right_res = self.compute_ik_srv(right_req)
                self.right_arm_joint_names = right_res.solution.joint_state.name[-15:-8]
                self.right_arm_joint_positions = right_res.solution.joint_state.position[-15:-8]

                self.pub_joint_trajectory(self.left_arm_joint_positions, self.right_arm_joint_positions, duration=1./60.)


    def initial_position(self):
        self.is_initial_position = True
        left_goal = np.array([-0.22403730494976504, 1.0221862759800295, -0.004231173247533526,
                                -1.6816800555824925, 3.4992188107438276, -0.6937425604628604, -3.421731167902337])
        right_goal = np.array([0.097663966177584, 1.0444538126242922, 0.0020227937193020296,
                                -1.7221104815660215, 2.9980243476804516, -0.6872252646847725, -3.0321469897166784])

        self.pub_joint_trajectory(left_goal, right_goal, duration=10)
#-----------------------------------------------------------------------------------------------------------------
    def pub_joint_trajectory(self, left_joint_positions, right_joint_positions, duration=1./60.):
        left_joint_trajectory_msg = self.create_joint_trajectory_msg(self.left_arm_joint_names, left_joint_positions, duration)
        right_joint_trajectory_msg = self.create_joint_trajectory_msg(self.right_arm_joint_names, right_joint_positions, duration)
        self.left_joint_pub.publish(left_joint_trajectory_msg)
        self.right_joint_pub.publish(right_joint_trajectory_msg)


def main():
    arm = MovePR2Arm()

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        arm.move_both_arms(rospy.Time.now())
        rate.sleep()

    # print(k)

if __name__ == '__main__':
    rospy.init_node("pr2_moveit_online")
    main()

