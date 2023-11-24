#! /usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import JointState


class RosbagCheck():
    def __init__(self):
        rospy.init_node("rosbag_check_node")
        rospy.loginfo("rosbag_check_node start")

        npz = np.load("/home/genki/dataset/test/eipl_tutorial_1.npz")
        self.left_joints = npz["left_joints"]
        self.left_gripper = npz["left_gripper"]
        self.right_joints = npz["right_joints"]
        self.right_gripper = npz["right_gripper"]
        self.target_pos = np.concatenate([self.left_joints, self.left_gripper.reshape(-1,1), self.right_joints, self.right_gripper.reshape(-1,1)], axis=1)
        
        self.config()

        self.publisher = rospy.Publisher("/target_position", JointState, queue_size=10)
        while self.publisher.get_num_connections() == 0:
            rospy.sleep(1)

    def config(self):
        self.left_arm_name = ['l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 
                              'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'left_gripper']
        self.right_arm_name = ['r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 
                              'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint','right_gripper']
        self.target_name = self.left_arm_name + self.right_arm_name

        self.target_joint_state = JointState()
        self.target_joint_state.name = self.target_name

    def run(self):
        rate = rospy.Rate(10)

        rospy.loginfo("start the motion")
        for i in range(len(self.target_pos)):
            print(self.target_pos[i])
            self.target_joint_state.position = self.target_pos[i]
            self.target_joint_state.header.stamp = rospy.Time.now()
            self.publisher.publish(self.target_joint_state)
            rate.sleep()

        rospy.loginfo("finish the motion")
        


if __name__ == "__main__":
    try:
        rosbag_check = RosbagCheck()
        rosbag_check.run()
    except rospy.ROSInitException:
        pass