#! /usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class InitialPosition:
    def __init__(self):
        rospy.init_node("initial_pos_node")
        rospy.loginfo("initial_pos_node start")

        self.torso_joint_name = ["torso_lift_joint"]
        self.head_joint_name = ["head_tilt_joint", "head_pan_joint"]
        self.left_arm_joint_names = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                                    'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        self.right_arm_joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                                    'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
        
        self.torso_initial_position = np.array([0.05])
        self.head_initial_position = np.array([0.65, 0.0])
        self.left_initial_position = np.array([0.2456659831018584, 0.656250503218148, -0.004510434091650417,
                                                -1.6764822051123867, 2.8524502921772603, -1.0396782712307289, -2.9883839686551203])
        self.right_initial_position = np.array([-0.1352291944311661, 0.785688780765561, 0.0019417232535294817,
                                                -1.7594344089689882, -2.9777877818255325, -0.9800244541303877, 3.0484269197453857])

        self.torso_pub = rospy.Publisher("/torso_controller/command", JointTrajectory, queue_size=10)
        self.head_pub = rospy.Publisher("/head_traj_controller/command", JointTrajectory, queue_size=10)
        self.left_joint_pub = rospy.Publisher("/l_arm_controller/command", JointTrajectory, queue_size=10)
        self.right_joint_pub = rospy.Publisher("/r_arm_controller/command", JointTrajectory, queue_size=10)
    
    def create_joint_trajectory_msg(self, joint_names, joint_positions, duration=5):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header = Header()
        joint_trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(duration)
        joint_trajectory.points.append(point)    
        return joint_trajectory
    
    def run(self):
        rospy.logwarn("Move to initial")
        
        torso_initial_trajectory = self.create_joint_trajectory_msg(self.torso_joint_name, self.torso_initial_position, duration=5)
        head_initial_pos_trajectory = self.create_joint_trajectory_msg(self.head_joint_name, self.head_initial_position, duration=5)
        left_initial_pos_trajectory = self.create_joint_trajectory_msg(self.left_arm_joint_names, self.left_initial_position, duration=5)
        right_initial_pos_trajectory = self.create_joint_trajectory_msg(self.right_arm_joint_names, self.right_initial_position, duration=5)

        self.torso_pub.publish(torso_initial_trajectory) 
        self.head_pub.publish(head_initial_pos_trajectory)
        self.left_joint_pub.publish(left_initial_pos_trajectory)
        self.right_joint_pub.publish(right_initial_pos_trajectory)
        
        rospy.sleep(5)
        rospy.logwarn("Reach to initial position")

if __name__ == "__main__":
    initial_position = InitialPosition()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        initial_position.run()
        break