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
    cmds = []
    images = []
    seq_length = []

    files = glob.glob(os.path.join(dir, "*.npz"))
    files.sort()
    for filename in files:
        print(filename)
        npz_data = np.load(filename)

        images.append(resize_img(npz_data["images"][:-skip], (128, 128)))
        finger_state = cos_interpolation(npz_data["right_gripper"])
        _joints = np.concatenate((npz_data["right_joints"][:-skip], finger_state[:-skip]), axis=-1)
        joints.append(_joints)
        _cmds = np.concatenate((npz_data["right_cmds"][:-skip], finger_state[:-skip]), axis=-1)
        cmds.append(_cmds)
        seq_length.append(len(_joints))

    max_seq = max(seq_length)
    images = list_to_numpy(images, max_seq)
    joints = list_to_numpy(joints, max_seq)
    cmds = list_to_numpy(cmds, max_seq)

    return images, joints, cmds


if __name__ == "__main__":
    # dataset index
    train_list = [0,1,3, 5,6,8,9, 10,11,13,14]
    test_list = [2, 4, 7, 12, 15]
    root_dir = "/home/genki/ros/workspace/pr2_ws/bags/eipl_tutorial/"

    # load data
    npz_dir = root_dir + "npz/"
    images, joints, cmds = load_data(npz_dir)

    # save images and joints
    train_dir = root_dir + "data/train"
    test_dir = root_dir + "data/test"
    if not os.path.exists(train_dir):
        os.makedirs(train_dir)
    if not os.path.exists(test_dir):
        os.makedirs(test_dir)

    np.save(root_dir + "data/train/images.npy", images[train_list].astype(np.uint8))
    np.save(root_dir + "data/train/joints.npy", joints[train_list].astype(np.float32))
    np.save(root_dir + "data/test/images.npy", images[test_list].astype(np.uint8))
    np.save(root_dir + "data/test/joints.npy", joints[test_list].astype(np.float32))

    # save joint bounds
    joint_bounds = calc_minmax(joints)
    np.save(root_dir + "data/joint_bounds.npy", joint_bounds)
