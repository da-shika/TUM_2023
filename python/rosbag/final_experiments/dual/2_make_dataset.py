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
 

def main():
    npz_root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/final_experiment/npz/dual"
    train_list = [5,6,8,9, 10,11,13,14, 15,16,18,19, 20,21,23,24, 25,26,28,29,
                  36,37,39,40, 41,42,44,45, 46,47,49,50, 51,52,54,55, 56,57,59,60, 61,62,64,65,
                  66,67,69,70, 71,72,74,75, 76,77,79,80, 81,82,84,85, 86,87,89,90, 91,92,94,95, 102,103,105,106]
    
    test_list = [0,1,2,3,4, 7, 12, 17, 22, 27, 30,31,32,33,34,35,
                 38, 43, 48, 53, 58, 63, 
                 68, 73, 78, 83, 88, 93, 96,97,98,99,100,101,104]

    left_images, right_images, left_joints, right_joints, \
    left_poses, right_poses, left_forces_raw, right_forces_raw, left_proximities_raw, right_proximities_raw = load_data(npz_root_dir)

    _left_force = np.delete(left_forces_raw, [8], axis=2)
    
    left_forces = np.max(_left_force, axis=2, keepdims=True)
    right_forces = np.max(right_forces_raw, axis=2, keepdims=True)
    left_proximities = np.max(left_proximities_raw, axis=2, keepdims=True)
    right_proximities = np.max(right_proximities_raw, axis=2, keepdims=True)
    
    # save images and joints
    root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/final_experiment/data/dual"
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

        np.save(root_dir + "/"+ mode +"/left_forces_raw.npy", left_forces_raw[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/left_forces.npy", left_forces[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/left_proximities_raw.npy", left_proximities_raw[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/left_proximities.npy", left_proximities[mode_list].astype(np.float32))

        np.save(root_dir + "/"+ mode +"/right_images.npy", right_images[mode_list].astype(np.uint8))
        np.save(root_dir + "/"+ mode +"/right_joints.npy", right_joints[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_poses.npy", right_poses[mode_list].astype(np.float32))

        np.save(root_dir + "/"+ mode +"/right_forces_raw.npy", right_forces_raw[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_forces.npy", right_forces[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_proximities_raw.npy", right_proximities_raw[mode_list].astype(np.float32))
        np.save(root_dir + "/"+ mode +"/right_proximities.npy", right_proximities[mode_list].astype(np.float32))
    
    # save data
    saver("train", train_list)
    saver("test", test_list)

    # save bounds
    left_joint_bounds = calc_minmax(left_joints)
    right_joint_bounds = calc_minmax(right_joints)
    left_pos_bounds = calc_minmax(left_poses)
    right_pos_bounds = calc_minmax(right_poses)

    left_force_bounds_raw = calc_minmax(left_forces_raw)
    right_force_bounds_raw = calc_minmax(right_forces_raw)
    left_force_bounds = calc_minmax(left_forces)
    right_force_bounds = calc_minmax(right_forces)
    left_proximity_bounds_raw = calc_minmax(left_proximities_raw)
    right_proximity_bounds_raw = calc_minmax(right_proximities_raw)
    left_proximity_bounds = calc_minmax(left_proximities)
    right_proximity_bounds = calc_minmax(right_proximities)

    np.save(root_dir + "/left_joint_bounds.npy", left_joint_bounds)
    np.save(root_dir + "/right_joint_bounds.npy", right_joint_bounds)
    np.save(root_dir + "/left_pose_bounds.npy", left_pos_bounds)
    np.save(root_dir + "/right_pose_bounds.npy", right_pos_bounds)
    
    np.save(root_dir + "/left_force_bounds_raw.npy", left_force_bounds_raw)
    np.save(root_dir + "/right_force_bounds_raw.npy", right_force_bounds_raw)
    np.save(root_dir + "/left_force_bounds.npy", left_force_bounds)
    np.save(root_dir + "/right_force_bounds.npy", right_force_bounds)
    np.save(root_dir + "/left_proximity_bounds_raw.npy", left_proximity_bounds_raw)
    np.save(root_dir + "/right_proximity_bounds_raw.npy", right_proximity_bounds_raw)
    np.save(root_dir + "/left_proximity_bounds.npy", left_proximity_bounds)
    np.save(root_dir + "/right_proximity_bounds.npy", right_proximity_bounds)

if __name__ == "__main__":
    main()