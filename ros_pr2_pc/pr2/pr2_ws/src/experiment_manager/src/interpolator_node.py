#! /usr/bin/env python

import sys
import numpy as np
from enum import Enum

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from experiment_manager.hand_controller import PR2GripperController

PI = np.pi

def linear_interpolation(q0, q1, T, t):
    """Linear interpolation between q0 and q1
    """
    s = min(1.0, max(0.0, t / T))
    return (q1 - q0)*s + q0

def create_joint_trajectory(joint_name):
    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = joint_name
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Time(0)
    trajectory.points.append(point)
    return trajectory


class Interpolator:
    class Mode(Enum):
        NONE = 1
        LINEAR = 2

    def __init__(self, controll_freq, target_freq, max_target_jump, mode=Mode.LINEAR):
        rospy.init_node("interpolator_node")
        rospy.loginfo("interpolator_node start")

        self.controll_freq = controll_freq
        self.max_target_delay = 5*1./controll_freq
        self.target_period = 1./ target_freq
        self.max_target_jump = max_target_jump
        self.mode = mode
        self.config()

        # setup robot
        self.hand_controller = PR2GripperController()

        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.target_sub = rospy.Subscriber("/target_position", JointState, self.target_callback)
        self.left_arm_pub = rospy.Publisher("/l_arm_controller/command", JointTrajectory, queue_size=10)
        self.right_arm_pub = rospy.Publisher("/r_arm_controller/command", JointTrajectory, queue_size=10)

        while self.left_arm_pub.get_num_connections() == 0 and self.right_arm_pub.get_num_connections() == 0:
            rospy.sleep(1)

    def config(self):
        self.left_arm_name = ['l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 
                              'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        self.left_arm_current = None
        self.left_arm_target = None
        self.left_gripper_target = None
        self.left_pub_cmd = None
        self.left_prev_target = None

        self.right_arm_name = ['r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 
                              'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
        self.right_arm_current = None
        self.right_arm_target = None
        self.right_gripper_target = None
        self.right_pub_cmd = None
        self.right_prev_target = None

        self.is_valid_target = False
        self.target_update_time = None

    def joint_state_callback(self, msg):
        self.left_arm_current = np.array(msg.position[-14:-7])
        self.right_arm_current = np.array(msg.position[17:24])

    def target_callback(self, msg):
        self.left_arm_target = np.array(msg.position[0:7])
        self.left_gripper_target = msg.position[7]
        self.right_arm_target = np.array(msg.position[8:15])
        self.right_gripper_target = msg.position[15]
        self.target_update_time = rospy.Time.now()

        if self.left_arm_target is not None and self.right_arm_target is not None:
            # check target jump
            if (np.max(np.abs(self.left_arm_current - self.left_arm_target)) > self.max_target_jump or \
                np.max(np.abs(self.right_arm_current - self.right_arm_target)) > self.max_target_jump):
                self.is_valid_target = False
                rospy.logerr_throttle(0.5, "Too big jump")
                return

            self.hand_controll()
            
            self.is_valid_target = True

#-------------------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.controll_freq)

        rospy.logwarn("Interpolator start")
        while not rospy.is_shutdown():
            self.arm_controll()
            rate.sleep()
        
        rospy.logwarn("shutdown")

#-------------------------------------------------------------------------------
    def arm_controll(self):
        if self.left_arm_current is not None and self.right_arm_current is not None:
            # make target position (chose interpolate or not)
            if not self.is_valid_target:
                rospy.logwarn_throttle(0.5, "OnlineExecutor.run(): Wait for first target")
                self.left_pub_cmd = self.left_arm_current
                self.right_pub_cmd = self.right_arm_current
                self.left_prev_target = np.copy(self.left_arm_current)
                self.right_prev_target = np.copy(self.right_arm_current)

            if self.is_valid_target:
                rospy.logwarn_throttle(0.5, "Valuable target")
                t = (rospy.Time.now() - self.target_update_time).to_sec()
                if self.mode == Interpolator.Mode.LINEAR:
                    self.left_pub_cmd = linear_interpolation(self.left_prev_target, self.left_arm_target, self.target_period, t)
                    self.right_pub_cmd = linear_interpolation(self.right_prev_target, self.right_arm_target, self.target_period, t)
                elif self.mode == Interpolator.Mode.NONE:
                    self.left_pub_cmd = self.left_arm_target
                    self.right_pub_cmd = self.right_arm_target

                if t > self.max_target_delay:
                    self.is_valid_target = False
                    rospy.logwarn("OnlineExecutor.run(): Interpolation stopped, wait for valid command")
        
                self.publish()
    

    def hand_controll(self):
        if self.left_gripper_target == 1:
            self.hand_controller.left_open()
        elif self.left_gripper_target == 0:
            self.hand_controller.left_close()

        if self.right_gripper_target == 1:
            self.hand_controller.right_open()
        elif self.right_gripper_target == 0:
            self.hand_controller.right_close()

    def publish(self):
        self.left_trajectory = create_joint_trajectory(self.left_arm_name)
        self.right_trajectory = create_joint_trajectory(self.right_arm_name)
        if not np.isnan(self.left_pub_cmd).any():
            self.left_trajectory.header.stamp = rospy.Time.now()
            self.left_trajectory.points[0].positions = self.left_pub_cmd.tolist()
            self.left_arm_pub.publish(self.left_trajectory)

        if not np.isnan(self.right_pub_cmd).any():
            self.right_trajectory.header.stamp = rospy.Time.now()
            self.right_trajectory.points[0].positions = self.right_pub_cmd.tolist()
            self.right_arm_pub.publish(self.right_trajectory)

#-------------------------------------------------------------------------------
if __name__ == "__main__":
    interpolator = Interpolator(
        controll_freq=30,
        target_freq=10,
        max_target_jump=360./180.*PI
    )

    interpolator.run()