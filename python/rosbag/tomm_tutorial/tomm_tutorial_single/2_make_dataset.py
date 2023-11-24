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
    joints = []
    poses = []
    forces = []
    proximities = []
    images = []
    seq_length = []

    files = glob.glob(os.path.join(dir, "*.npz"))
    files.sort()
    for filename in files:
        print(filename)
        npz_data = np.load(filename)

        images.append(resize_img(npz_data["images"][:-skip], (128, 128)))
        joints.append(npz_data["joints"][:-skip])
        poses.append(npz_data["poses"][:-skip])
        forces.append(npz_data["forces"][:-skip])
        proximities.append(npz_data["proximities"][:-skip])
        
        seq_length.append(len(npz_data["images"][:-skip]))
    
    max_seq = max(seq_length)
    images = list_to_numpy(images, max_seq)
    joints = list_to_numpy(joints, max_seq)
    poses = list_to_numpy(poses, max_seq)
    forces = list_to_numpy(forces, max_seq)
    proximities = list_to_numpy(proximities, max_seq)

    return images, joints, poses, forces, proximities


if __name__ == "__main__":
    # dataset index
    train_list = [0,1,3,4, 6,7,9,10, 11,12,14,15]
    test_list = [2, 5, 8, 13, 16]
    root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial"

    # load data
    npz_dir = os.path.join(root_dir, "npz/single")
    images, joints, poses, forces, proximities = load_data(npz_dir)

    # save images and joints
    train_dir = os.path.join(root_dir, "data/single/train")
    test_dir = os.path.join(root_dir, "data/single/test")
    if not os.path.exists(train_dir):
        os.makedirs(train_dir)
    if not os.path.exists(test_dir):
        os.makedirs(test_dir)
    
    def saver(mode, mode_list):
        np.save(root_dir + "/data/single/"+ mode +"/images.npy", images[mode_list].astype(np.uint8))
        np.save(root_dir + "/data/single/"+ mode +"/joints.npy", joints[mode_list].astype(np.float32))
        np.save(root_dir + "/data/single/"+ mode +"/poses.npy", poses[mode_list].astype(np.float32))
        np.save(root_dir + "/data/single/"+ mode +"/forces.npy", forces[mode_list].astype(np.float32))
        np.save(root_dir + "/data/single/"+ mode +"/proximities.npy", proximities[mode_list].astype(np.float32))
    
    # save data
    saver("train", train_list)
    saver("test", test_list)

    # save bounds
    joint_bounds = calc_minmax(joints)
    pos_bounds = calc_minmax(poses)
    force_bounds = calc_minmax(forces)
    proximity_bounds = calc_minmax(proximities)

    bound_save_dir = root_dir + "/data/single/"
    np.save(bound_save_dir + "joint_bounds.npy", joint_bounds)
    np.save(bound_save_dir + "pose_bounds.npy", pos_bounds)
    np.save(bound_save_dir + "force_bounds.npy", force_bounds)
    np.save(bound_save_dir + "proximity_bounds.npy", proximity_bounds)
