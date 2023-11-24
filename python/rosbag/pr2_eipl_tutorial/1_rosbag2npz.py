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


parser = argparse.ArgumentParser()
parser.add_argument("--bag_dir", type=str)
parser.add_argument("--freq", type=float, default=15)
args = parser.parse_args()


files = glob.glob(os.path.join(args.bag_dir, "*.bag"))
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
        "/joint_states",
        "/r_arm_controller/command",
        "/pr2_left_gripper_cmd",
        "/pr2_right_gripper_cmd",
        "/camera/rgb/image_compressed"
    ]

    # Create a rospy.Time object to represent the current time
    current_time = rospy.Time.from_sec(start_time)

    left_joint_list = []
    right_joint_list = []
    right_cmd_list = []
    left_gripper_list = []
    right_gripper_list = []
    image_list = []

    # Loop through the rosbag file at regular intervals (args.freq)
    freq = 1.0 / float(args.freq)
    while current_time.to_sec() < end_time:
        print(current_time.to_sec())

        # Get the messages for each topic at the current time
        for topic in topics:
            for topic_msg, msg, time in bag.read_messages(topic):
                if time >= current_time:
                    if topic == "/joint_states":
                        left_joint_list.append(msg.position[-14:-7])
                        right_joint_list.append(msg.position[17:24])
                    
                    if topic == "/r_arm_controller/command":
                        right_cmd_list.append(msg.points[0].positions)
                    
                    if topic == "/pr2_left_gripper_cmd":
                        left_gripper_list.append(msg.data)

                    if topic == "/pr2_right_gripper_cmd":
                        right_gripper_list.append(msg.data)

                    if topic == "/camera/rgb/image_compressed":
                        crop_x1 = int(285/2)
                        crop_y1 = int(235/2)
                        x_crop_size = int(230/2)
                        y_crop_size = int(230/2)
                        np_arr = np.frombuffer(msg.data, np.uint8)
                        np_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        np_img = np_img[::2, ::2]
                        image_list.append(np_img[crop_y1:crop_y1+y_crop_size, crop_x1:crop_x1+x_crop_size].astype(np.uint8))

                    break

        # Wait for the next interval
        current_time += rospy.Duration.from_sec(freq)
        rospy.sleep(freq)

    # Close the rosbag file
    bag.close()

    # Convert list to array
    left_joints = np.array(left_joint_list, dtype=np.float32)
    right_joints = np.array(right_joint_list, dtype=np.float32)
    right_cmds = np.array(right_cmd_list, dtype=np.float32)
    left_gripper = np.array(left_gripper_list, dtype=np.float32)
    right_gripper = np.array(right_gripper_list, dtype=np.float32)
    images = np.array(image_list, dtype=np.uint8)

    # Get shorter lenght
    shorter_length = min(len(left_joints), len(right_joints), 
                         len(right_cmds), 
                         len(images), 
                         len(left_gripper), len(right_gripper))

    # Trim
    left_joints = left_joints[:shorter_length]
    right_joints = right_joints[:shorter_length]
    right_cmds = right_cmds[:shorter_length]
    left_gripper = left_gripper[:shorter_length]
    right_gripper = right_gripper[:shorter_length]
    images = images[:shorter_length]
    
    # Save
    np.savez(savename, 
             left_joints=left_joints, right_joints=right_joints, 
             right_cmds=right_cmds, 
             left_gripper=left_gripper, right_gripper=right_gripper, 
             images=images)
