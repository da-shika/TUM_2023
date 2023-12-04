#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import os
import cv2
import glob
import rospy
import rosbag
import argparse
import numpy as np
from geometry_msgs.msg import PoseStamped

parser = argparse.ArgumentParser()
parser.add_argument("--dir", type=str)
parser.add_argument("--freq", type=float, default=10)
args = parser.parse_args()


files = glob.glob(os.path.join(args.dir, "*.bag"))
files.sort()
for file in files:
    print(file)
    savename = file.split(".bag")[0] + ".npz"

    # Open the rosbag file
    bag = rosbag.Bag(file)

    # Get the start and end times of the rosbag file
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()

    # Get the topics in the rosbag file
    # topics = bag.get_type_and_topic_info()[1].keys()
    topics = [
        "/tomm/joint_states",
        "/tomm/teleop_left_hand/pose", "/tomm/teleop_right_hand/pose",
        "/tomm/arm_right/hand/left_hand_back/data", "/tomm/arm_right/hand/right_hand_back/data",
        "/usb_camera/republished/compressed"
    ]

    # Create a rospy.Time object to represent the current time
    current_time = rospy.Time.from_sec(start_time)

    left_joint_list, right_joint_list = [], []
    left_pos_list, right_pos_list = [], []
    left_force_list, right_force_list = [], []
    left_proximity_list, right_proximity_list = [], []
    left_image_list, right_image_list = [], []

    # Loop through the rosbag file at regular intervals (args.freq)
    freq = 1.0 / float(args.freq)
    while current_time.to_sec() < end_time:
        print(current_time.to_sec())

        # Get the messages for each topic at the current time
        for topic in topics:
            for topic_msg, msg, time in bag.read_messages(topic):
                if time >= current_time:
                    if topic == "/tomm/joint_states":
                        left_joint_list.append(msg.position[3:9])
                        right_joint_list.append(msg.position[9:15])
                    
                    if topic == "/tomm/teleop_left_hand/pose":
                        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                        left_pos_list.append(pos)
                    if topic == "/tomm/teleop_right_hand/pose":
                        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                        right_pos_list.append(pos)

                    if topic == "/tomm/arm_right/hand/left_hand_back/data":
                        left_force_list.append(msg.force)
                    if topic == "/tomm/arm_right/hand/right_hand_back/data":
                        right_force_list.append(msg.force)

                    if topic == "/tomm/arm_right/hand/left_hand_back/data":
                        left_proximity_list.append(msg.prox)
                    if topic == "/tomm/arm_right/hand/right_hand_back/data":
                        right_proximity_list.append(msg.prox)

                    if topic == "/usb_camera/republished/compressed":
                        crop_x1 = int(100/2)
                        crop_y1 = int(120/2)
                        crop_size = int(320/2)
                        np_arr = np.frombuffer(msg.data, np.uint8)
                        np_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        np_img = np_img[::2, ::2]
                        left_image_list.append(np_img[crop_y1 : crop_y1+crop_size, crop_x1 : crop_x1+crop_size].astype(np.uint8))
                        right_image_list.append(np_img[crop_y1 : crop_y1+crop_size, crop_x1+crop_size : crop_x1+2*crop_size].astype(np.uint8))
                        
                    break

        # Wait for the next interval
        current_time += rospy.Duration.from_sec(freq)
        rospy.sleep(freq)

    # Close the rosbag file
    bag.close()

    # Convert list to array
    left_joints = np.array(left_joint_list, dtype=np.float32)
    right_joints = np.array(right_joint_list, dtype=np.float32)
    left_poses = np.array(left_pos_list, dtype=np.float32)
    right_poses = np.array(right_pos_list, dtype=np.float32)
    left_forces = np.array(left_force_list, dtype=np.float32)
    right_forces = np.array(right_force_list, dtype=np.float32)
    left_proximities = np.array(left_proximity_list, dtype=np.float32)
    right_proximities = np.array(right_proximity_list, dtype=np.float32)
    left_images = np.array(left_image_list, dtype=np.uint8)
    right_images = np.array(right_image_list, dtype=np.uint8)

    # Get shorter lenght
    shorter_length = min(len(left_joints), len(right_joints), 
                         len(left_poses), len(right_poses),
                         len(left_forces), len(right_forces),
                         len(left_proximities), len(right_proximities),
                         len(left_images), len(right_images))

    # Trim
    left_joints = left_joints[:shorter_length]
    right_joints = right_joints[:shorter_length]
    left_poses = left_poses[:shorter_length]
    right_poses = right_poses[:shorter_length]
    left_forces = left_forces[:shorter_length]
    right_forces = right_forces[:shorter_length]
    left_proximities = left_proximities[:shorter_length]
    right_proximities = right_proximities[:shorter_length]
    left_images = left_images[:shorter_length]
    right_images = right_images[:shorter_length]
    
    # Save
    np.savez(savename, 
             left_joints=left_joints, right_joints=right_joints, 
             left_poses=left_poses, right_poses=right_poses, 
             left_forces=left_forces, right_forces=right_forces, 
             left_proximities=left_proximities, right_proximities=right_proximities, 
             left_images=left_images, right_images=right_images)
