#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

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


if __name__ == "__main__":
    # dataset index
    train_list = [0,1,3,4,6,7,9,10, 11,12,14,15,16,17,19,20,
                  21,22,24,25,28,29, 31,32]
    test_list = [2,5,8, 13,18, 23,26,27, 30,33,34,35]
    root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate"

    # load data
    npz_dir = os.path.join(root_dir, "npz/dual")
    left_images, right_images, left_joints, right_joints, \
    left_poses, right_poses, left_forces, right_forces, left_proximities, right_proximities = load_data(npz_dir)

    # save images and joints
    train_dir = os.path.join(root_dir, "data/dual/train")
    test_dir = os.path.join(root_dir, "data/dual/test")
    if not os.path.exists(train_dir):
        os.makedirs(train_dir)
    if not os.path.exists(test_dir):
        os.makedirs(test_dir)
    
    def saver(mode, mode_list):
        np.save(root_dir + "/data/dual/"+ mode +"/left_images.npy", left_images[mode_list].astype(np.uint8))
        np.save(root_dir + "/data/dual/"+ mode +"/left_joints.npy", left_joints[mode_list].astype(np.float32))
        np.save(root_dir + "/data/dual/"+ mode +"/left_poses.npy", left_poses[mode_list].astype(np.float32))
        np.save(root_dir + "/data/dual/"+ mode +"/left_forces_raw.npy", left_forces[mode_list].astype(np.float32))
        np.save(root_dir + "/data/dual/"+ mode +"/left_proximities.npy", left_proximities[mode_list].astype(np.float32))
        np.save(root_dir + "/data/dual/"+ mode +"/right_images.npy", right_images[mode_list].astype(np.uint8))
        np.save(root_dir + "/data/dual/"+ mode +"/right_joints.npy", right_joints[mode_list].astype(np.float32))
        np.save(root_dir + "/data/dual/"+ mode +"/right_poses.npy", right_poses[mode_list].astype(np.float32))
        np.save(root_dir + "/data/dual/"+ mode +"/right_forces_raw.npy", right_forces[mode_list].astype(np.float32))
        np.save(root_dir + "/data/dual/"+ mode +"/right_proximities.npy", right_proximities[mode_list].astype(np.float32))
    
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

    np.save(root_dir + "/data/dual/left_joint_bounds.npy", left_joint_bounds)
    np.save(root_dir + "/data/dual/right_joint_bounds.npy", right_joint_bounds)
    np.save(root_dir + "/data/dual/left_pose_bounds.npy", left_pos_bounds)
    np.save(root_dir + "/data/dual/right_pose_bounds.npy", right_pos_bounds)
    np.save(root_dir + "/data/dual/left_force_bounds_raw.npy", left_force_bounds)
    np.save(root_dir + "/data/dual/right_force_bounds_raw.npy", right_force_bounds)
    np.save(root_dir + "/data/dual/left_proximity_bounds.npy", left_proximity_bounds)
    np.save(root_dir + "/data/dual/right_proximity_bounds.npy", right_proximity_bounds)
