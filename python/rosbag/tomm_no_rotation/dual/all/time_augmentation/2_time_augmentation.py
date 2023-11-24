#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import os
import cv2
import glob
import random
import argparse
import numpy as np
import matplotlib.pylab as plt
from eipl.utils import resize_img, calc_minmax, list_to_numpy, cos_interpolation

import os
import cv2
import glob
import argparse
import numpy as np
import matplotlib.pylab as plt
from eipl.utils import resize_img, calc_minmax, list_to_numpy, cos_interpolation


def load_data(dir):
    skip = 10
    left_joints, right_joints = [], []
    left_poses, right_poses = [], []
    left_forces, right_forces = [], []
    left_proximities, right_proximities = [], []
    left_images, right_images = [], []
    seq_length = []

    files = glob.glob(os.path.join(dir, "*.npz"))
    files.sort()
    for filename in files:
        print(filename)
        npz_data = np.load(filename)

        left_images.append(resize_img(npz_data["left_images"][:-skip], (128, 128)))
        right_images.append(resize_img(npz_data["right_images"][:-skip], (128, 128)))
        left_joints.append(npz_data["left_joints"][:-skip])
        right_joints.append(npz_data["right_joints"][:-skip])

        left_poses.append(npz_data["left_poses"][:-skip])
        right_poses.append(npz_data["right_poses"][:-skip])
        left_forces.append(npz_data["left_forces"][:-skip])
        right_forces.append(npz_data["right_forces"][:-skip])
        left_proximities.append(npz_data["left_proximities"][:-skip])
        right_proximities.append(npz_data["right_proximities"][:-skip])
        
        seq_length.append(len(npz_data["left_images"][:-skip]))

    max_seq = max(seq_length)
    left_images = list_to_numpy(left_images, max_seq)
    right_images = list_to_numpy(right_images, max_seq)
    left_joints = list_to_numpy(left_joints, max_seq)
    right_joints = list_to_numpy(right_joints, max_seq)

    left_poses = list_to_numpy(left_poses, max_seq)
    right_poses = list_to_numpy(right_poses, max_seq)
    left_forces = list_to_numpy(left_forces, max_seq)
    right_forces = list_to_numpy(right_forces, max_seq)
    left_proximities = list_to_numpy(left_proximities, max_seq)
    right_proximities = list_to_numpy(right_proximities, max_seq)

    return left_images, right_images, left_joints, right_joints, \
            left_poses, right_poses, left_forces, right_forces, left_proximities, right_proximities


def time_augmentation(npz_root_dir):
    left_joints_list, right_joints_list = [], []
    left_poses_list, right_poses_list = [], []
    left_forces_list, right_forces_list = [], []
    left_proximities_list, right_proximities_list = [], []
    left_images_list, right_images_list = [], []

    # load data
    npz_dir = os.path.join(npz_root_dir, "npz/dual")
    left_images, right_images, left_joints, right_joints, \
    left_poses, right_poses, left_forces, right_forces, left_proximities, right_proximities = load_data(npz_dir)

    def _time_augmentation_image(image, extend_time_step):
        front_extension = np.tile(image[0:1], (extend_time_step,1,1,1))
        end_extension = np.tile(image[-1:], (30-extend_time_step,1,1,1))
        extend_image = np.concatenate([front_extension, image, end_extension])
        return extend_image
    
    def _time_augmentation_signal(data, extend_time_step):
        front_extension = np.tile(data[0:1], (extend_time_step,1))
        end_extension = np.tile(data[-1:], (30-extend_time_step,1))
        extend_data = np.concatenate([front_extension, data, end_extension])
        return extend_data

    for i in range(len(left_images)):
        extend_time_step = random.choice([0, 10, 20, 30])

        left_images_list.append(_time_augmentation_image(left_images[i], extend_time_step))
        right_images_list.append(_time_augmentation_image(right_images[i], extend_time_step))
        left_joints_list.append(_time_augmentation_signal(left_joints[i], extend_time_step))
        right_joints_list.append(_time_augmentation_signal(right_joints[i], extend_time_step))

        left_poses_list.append(_time_augmentation_signal(left_poses[i], extend_time_step))
        right_poses_list.append(_time_augmentation_signal(right_poses[i], extend_time_step))
        left_forces_list.append(_time_augmentation_signal(left_forces[i], extend_time_step))
        right_forces_list.append(_time_augmentation_signal(right_forces[i], extend_time_step))
        left_proximities_list.append(_time_augmentation_signal(left_proximities[i], extend_time_step))
        right_proximities_list.append(_time_augmentation_signal(right_proximities[i], extend_time_step))

    left_images = np.array(left_images_list, dtype=np.uint8)
    right_images = np.array(right_images_list, dtype=np.uint8)
    left_joints = np.array(left_joints_list, dtype=np.float32)
    right_joints = np.array(right_joints_list, dtype=np.float32)
    left_poses = np.array(left_poses_list, dtype=np.float32)
    right_poses = np.array(right_poses_list, dtype=np.float32)
    left_forces = np.array(left_forces_list, dtype=np.float32)
    right_forces = np.array(right_forces_list, dtype=np.float32)
    left_proximities = np.array(left_proximities_list, dtype=np.float32)
    right_proximities = np.array(right_proximities_list, dtype=np.float32)

    return left_images, right_images, left_joints, right_joints, left_poses, right_poses, \
            left_forces, right_forces, left_proximities, right_proximities
        

def main():
    npz_root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate"
    train_list = [0,1,3,4,7,8,10, 11,12,13,15,16,17,18,20, 
                  21,22,23,25,26,29,30, 32,33]
    test_list = [2,5,6,9, 14,19, 24,27,28, 31,34,35,36]

    left_images, right_images, left_joints, right_joints, left_poses, right_poses, \
            left_forces, right_forces, left_proximities, right_proximities = time_augmentation(npz_root_dir)
    
    # save images and joints
    root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate/data/dual/all/time_augmention"
    train_dir = os.path.join(root_dir, "train")
    test_dir = os.path.join(root_dir, "test")
    if not os.path.exists(train_dir):
        os.makedirs(train_dir)
    if not os.path.exists(test_dir):
        os.makedirs(test_dir)
    
    def saver(mode, mode_list):
        np.save(root_dir + "/"+ mode +"/left_images.npy", left_images[mode_list].astype(np.uint8))
        np.save(root_dir + "/"+ mode +"/left_joints.npy", left_joints[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/left_poses.npy", left_poses[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/left_forces_raw.npy", left_forces[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/left_proximities.npy", left_proximities[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_images.npy", right_images[mode_list].astype(np.uint8))
        np.save(root_dir + "/"+ mode +"/right_joints.npy", right_joints[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_poses.npy", right_poses[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_forces_raw.npy", right_forces[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_proximities.npy", right_proximities[mode_list].astype(np.float32))
    
    # save data
    saver("train", train_list)
    saver("test", test_list)

    # save bounds
    left_joint_bounds = calc_minmax(left_joints)
    right_joint_bounds = calc_minmax(right_joints)
    left_pos_bounds = calc_minmax(left_poses)
    right_pos_bounds = calc_minmax(right_poses)
    left_force_bounds = calc_minmax(left_forces)
    right_force_bounds = calc_minmax(right_forces)
    left_proximity_bounds = calc_minmax(left_proximities)
    right_proximity_bounds = calc_minmax(right_proximities)

    np.save(root_dir + "/left_joint_bounds.npy", left_joint_bounds)
    np.save(root_dir + "/right_joint_bounds.npy", right_joint_bounds)
    np.save(root_dir + "/left_pose_bounds.npy", left_pos_bounds)
    np.save(root_dir + "/right_pose_bounds.npy", right_pos_bounds)
    np.save(root_dir + "/left_force_bounds_raw.npy", left_force_bounds)
    np.save(root_dir + "/right_force_bounds_raw.npy", right_force_bounds)
    np.save(root_dir + "/left_proximity_bounds.npy", left_proximity_bounds)
    np.save(root_dir + "/right_proximity_bounds.npy", right_proximity_bounds)

if __name__ == "__main__":
    main()