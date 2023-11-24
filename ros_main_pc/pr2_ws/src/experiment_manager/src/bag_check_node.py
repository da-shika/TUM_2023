#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from experiment_manager.hand_controller import PR2GripperController


def create_joint_trajectory(joint_names, hz):
    # set JointTrajectoryPoint message
    joint_trajectory = JointTrajectory()
    joint_trajectory.header = Header()
    joint_trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1/hz)
    joint_trajectory.points.append(point)    
    return joint_trajectory


class RosbagCheck:
    def __init__(self, hz, file_name):
        rospy.init_node("rosbag_check_node")
        rospy.loginfo("rosbag_check_node start")

        npz = np.load(file_name)
        self.left_joints = npz["left_joints"]
        self.left_gripper = npz["left_gripper"]
        self.right_joints = npz["right_joints"]
        self.right_gripper = npz["right_gripper"]

        self.left_arm_name = ['l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 
                              'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'left_gripper']
        self.right_arm_name = ['r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 
                              'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint','right_gripper']

        self.hand_controller = PR2GripperController()
        self.hz = hz

        self.left_arm_pub = rospy.Publisher("/l_arm_controller/command", JointTrajectory, queue_size=10)
        self.right_arm_pub = rospy.Publisher("/r_arm_controller/command", JointTrajectory, queue_size=10)
        while self.left_arm_pub.get_num_connections() == 0:
            rospy.sleep(1)
        self.left_trajectory = create_joint_trajectory(self.left_arm_name, self.hz)

        while self.right_arm_pub.get_num_connections() == 0:
            rospy.sleep(1)
        self.right_trajectory = create_joint_trajectory(self.right_arm_name, self.hz)


    def run(self):
        rate = rospy.Rate(self.hz)

        rospy.loginfo("start the motion")
        for i in range(len(self.left_joints)):
            self.left_trajectory.points[0].positions = self.left_joints[i]
            self.left_trajectory.header.stamp = rospy.Time.now()
            self.left_arm_pub.publish(self.left_trajectory)

            self.right_trajectory.points[0].positions = self.right_joints[i]
            self.right_trajectory.header.stamp = rospy.Time.now()
            self.right_arm_pub.publish(self.right_trajectory)

            if self.left_gripper[i] == 1:
                self.hand_controller.left_open()
            elif self.left_gripper[i] == 0:
                self.hand_controller.left_close()

            if self.right_gripper[i] == 1:
                self.hand_controller.right_open()
            elif self.right_gripper[i] == 0:
                self.hand_controller.right_close()

            rate.sleep()

        rospy.loginfo("finish the motion")
        


if __name__ == "__main__":
    dir_name = "/home/genki/ros/workspace/pr2_ws/bags/eipl_tutorial/l"
    bag_name = "eipl_tutorial_3.bag"
    file_name = dir_name + "/" + bag_name
    try:
        rosbag_check = RosbagCheck(hz=10, file_name=file_name)
        rosbag_check.run()
    except rospy.ROSInitException:
        pass