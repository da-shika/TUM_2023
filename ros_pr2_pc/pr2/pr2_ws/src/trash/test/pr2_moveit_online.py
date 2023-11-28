#!/usr/bin/env python
#-*- coding: utf-8 -*-

"""
Move by JointTrajectoryActionGoal
"""

import sys
import numpy as np

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from pr2_controllers_msgs.msg import JointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint

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
    
        self.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.wait_for_service("/compute_ik")

        self.robot_state = self.robot.get_current_state()
        self.t_start = rospy.Time.now()

        self.right_arm_config()
        self.pub_message_config()

        self.joint_pub = rospy.Publisher("/r_arm_controller/joint_trajectory_action/goal", JointTrajectoryActionGoal, queue_size=10)

    def right_arm_config(self):
        self.right_arm_joint_names = None
        self.righ_arm_joint_positions = None

    def pub_message_config(self):
        # set message
        self.joint_trajectory_goal = JointTrajectoryActionGoal()
        self.joint_trajectory_goal.goal_id.stamp = rospy.Time.now()
        self.joint_trajectory_goal.goal_id.id = "JointTrajectoryGoal"

        # set JointTrajectory message (is included JointTrajectoryActionGoal message)
        self.joint_trajectory_goal.goal.trajectory.header.stamp = rospy.Time.now()
        self.joint_trajectory_goal.goal.trajectory.joint_names = None

    def computeIK(self, time):
        
        # trajectory
        t = (time - self.t_start).to_sec()
        pose = trajectory(np.array([0.5, -0.1, 0.7]), 0.1, 0.1, t)

        # compute ik
        req = GetPositionIKRequest()
        req.ik_request.group_name = "right_arm"
        req.ik_request.robot_state = self.robot_state
        req.ik_request.avoid_collisions = False
        req.ik_request.pose_stamped = pose
        res = self.compute_ik_srv(req)

        # # move fake robot to ik
        self.robot_state = res.solution

        self.right_arm_joint_names = self.robot_state.joint_state.name[-15:-8]
        self.joint_trajectory_goal.goal.trajectory.joint_names = self.right_arm_joint_names
        self.righ_arm_joint_positions = self.robot_state.joint_state.position[-15:-8]
    
        # 1) pipe the solution into this topic /r_arm_controller/joint_trajectory_action/goal
        #    Question: How to set time_from_start: {secs: 0.1, nsecs: 0} ?

        # 2) use the function execute with a plan 
        #    self.arm_group.execute(plan_msg)
        #    Plan with single point (the next point only 1)
        #    moveit_msgs::RobotTrajectory <-- add a single point q_next

        self.pub_joint_trajectory(self.righ_arm_joint_positions)

    def pub_joint_trajectory(self, joint_positions, duration=0.1):
        # set JointTrajectoryPoint message
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(duration)
        self.joint_trajectory_goal.goal.trajectory.points.append(point)

        self.joint_pub.publish(self.joint_trajectory_goal)

def main():
    arm = MovePR2Arm()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        start = rospy.Time.now()
        arm.computeIK(rospy.Time.now())
        end = rospy.Time.now()
        rospy.logwarn_throttle(1.0, (end - start).to_sec())
        rate.sleep()

    # print(k)

if __name__ == '__main__':
    rospy.init_node("pr2_moveit_online")
    main()

