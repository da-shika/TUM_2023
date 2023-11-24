#! /usr/bin/env python3

import numpy as np
from enum import Enum

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PI = np.pi

def linear_interpolation(q0, q1, T, t):
    """Linear interpolation between q0 and q1
    """
    s = min(1.0, max(0.0, t / T))
    return (q1 - q0)*s + q0

def create_joint_trajectory(joint_name, controll_freq):
    trajectory = JointTrajectory()
    trajectory.header = Header()
    trajectory.joint_names = joint_name
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1/controll_freq)
    trajectory.points.append(point)
    return trajectory


class Interpolator:
    class Mode(Enum):
        NONE = 1
        LINEAR = 2

    def __init__(self, controll_freq, target_freq, max_target_jump, mode=Mode.LINEAR):
        self.controll_freq = controll_freq
        self.max_target_delay = 5*1./controll_freq
        self.target_period = 1./ target_freq
        self.max_target_jump = max_target_jump
        self.mode = mode
        self.config()

        self.joint_sub = rospy.Subscriber("/tomm/joint_states", JointState, self.joint_callback)
        self.target_sub = rospy.Subscriber("/target_command", JointState, self.target_callback)
        self.joint_pub = rospy.Publisher()    # TODO ask

        while self.left_arm_pub.get_num_connections() == 0 and self.right_arm_pub.get_num_connections() == 0:
            rospy.sleep(1)

    def config(self):
        self.joint_name = ["l_elbow_joint","l_shoulder_lift_joint","l_shoulder_pan_joint","l_wrist_1_joint","l_wrist_2_joint","l_wrist_3_joint", 
                            "r_elbow_joint","r_shoulder_lift_joint","r_shoulder_pan_joint","r_wrist_1_joint","r_wrist_2_joint","r_wrist_3_joint"]
   
        self.current_joint = None
        self.target_joint = None
        self.pub_cmd = None
        self.prev_target = None

        self.is_valid_target = False
        self.target_update_time = None

#---------------------------------------------------------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.controll_freq)

        rospy.logwarn("Interpolator start")
        while not rospy.is_shutdown():
            self.arm_controll()
            rate.sleep()
        
        rospy.logwarn("shutdown")

#---------------------------------------------------------------------------------------------------------------------
    def arm_controll(self):
        if self.current_joint is not None:
            # make target position (chose interpolate or not)
            if not self.is_valid_target:
                rospy.logwarn_throttle(0.5, "OnlineExecutor.run(): Wait for first target")
                self.pub_cmd = self.current_joint
                self.left_prev_target = np.copy(self.current_joint)

            if self.is_valid_target:
                rospy.logwarn_throttle(0.5, "Valuable target")
                t = (rospy.Time.now() - self.target_update_time).to_sec()
                if self.mode == Interpolator.Mode.LINEAR:
                    self.pub_cmd = linear_interpolation(self.left_prev_target, self.target_joint, self.target_period, t)
                elif self.mode == Interpolator.Mode.NONE:
                    self.pub_cmd = self.target_joint

                if t > self.max_target_delay:
                    self.is_valid_target = False
                    rospy.logwarn("OnlineExecutor.run(): Interpolation stopped, wait for valid command")
        
                self.publish()

    def publish(self):
        self.trajectory = create_joint_trajectory(self.joint_name, self.controll_freq)
        if not np.isnan(self.pub_cmd).any():
            self.trajectory.header.stamp = rospy.Time.now()
            self.trajectory.points[0].positions = self.pub_cmd.tolist()
            self.joint_pub.publish(self.trajectory)

#---------------------------------------------------------------------------------------------------------------------
    def joint_callback(self, msg):
        _left_joint = np.array(msg.position[3:9])
        _right_joint = np.array(msg.position[9:15])
        self.current_joint = np.concatenate((_left_joint, _right_joint), axis=0)

    def target_callback(self, msg):
        self.target_joint = np.array(msg.position[0:7])
        self.target_update_time = rospy.Time.now()

        if self.target_joint is not None:
            # check target jump
            if (np.max(np.abs(self.current_joint - self.target_joint)) > self.max_target_jump):
                self.is_valid_target = False
                rospy.logerr_throttle(0.5, "Too big jump")
                return
                        
            self.is_valid_target = True

#---------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node("interpolator_node")
    rospy.loginfo("interpolator_node start")

if __name__ == "__main__":
    interpolator = Interpolator(
        controll_freq=30,
        target_freq=10,
        max_target_jump=360000000000./180.*PI
    )

    interpolator.run()