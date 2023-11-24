#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import os
import argparse
import numpy as np
import matplotlib.pylab as plt
import matplotlib.animation as anim
from eipl.utils import normalization


parser = argparse.ArgumentParser()
parser.add_argument("--idx", type=int, default=0)
args = parser.parse_args()

idx = int(args.idx)
root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/data/single" + "/"

joints = np.load(root_dir + "test/joints.npy")
joint_bounds = np.load(root_dir + "joint_bounds.npy")
poses = np.load(root_dir + "test/poses.npy")
pose_bounds = np.load(root_dir + "pose_bounds.npy")

images = np.load(root_dir + "test/images.npy")

forces = np.load(root_dir + "test/forces.npy")
force_bounds = np.load(root_dir + "force_bounds.npy")
proximities = np.load(root_dir + "test/proximities.npy")
proximity_bounds = np.load(root_dir + "proximity_bounds.npy")

N = images.shape[1]


# normalized
minmax = [0.1, 0.9]
norm_joints = normalization( joints, joint_bounds, minmax )
norm_poses = normalization( poses, pose_bounds, minmax )
norm_forces = normalization( forces, force_bounds, minmax )
norm_proximities = normalization( proximities, proximity_bounds, minmax )

# print data information
print('load test data, index number is {}'.format(idx))
print('Joint: shape={}, min={:.3g}, max={:.3g}'.format(joints.shape, joints.min(), joints.max()) )
print('Norm Joint: shape={}, min={:.3g}, max={:.3g}'.format(norm_joints.shape, norm_joints.min(), norm_joints.max()) )

print('Pose: shape={}, min={:.3g}, max={:.3g}'.format(poses.shape, poses.min(), poses.max()) )
print('Norm Pose: shape={}, min={:.3g}, max={:.3g}'.format(norm_poses.shape, norm_poses.min(), norm_poses.max()) )

print('Force: shape={}, min={:.3g}, max={:.3g}'.format(forces.shape, forces.min(), forces.max()) )
print('Norm Force: shape={}, min={:.3g}, max={:.3g}'.format(norm_forces.shape, norm_forces.min(), norm_forces.max()) )

print('Proximity: shape={}, min={:.3g}, max={:.3g}'.format(proximities.shape, proximities.min(), proximities.max()) )
print('Norm Proximity: shape={}, min={:.3g}, max={:.3g}'.format(norm_proximities.shape, norm_proximities.min(), norm_proximities.max()) )

print('Image: shape={}'.format(images.shape) )


# plot images and normalized joints
fig, ax = plt.subplots(2, 5, figsize=(25, 16), dpi=60)

def anim_update(i):
    for j in range(2):
        for k in range(5):
            ax[j,k].cla()

    # plot left image
    ax[0,0].imshow(images[idx,i,:,:,::-1])
    ax[0,0].axis('off')
    ax[0,0].set_title('Image')

    def plot_data(ax_area, ylim_range, data, title):
        ax_area.set_ylim(ylim_range)
        ax_area.set_xlim(0, N)
        ax_area.plot(data[idx], linestyle='dashed', c='k')
        for joint_idx in range(data.shape[2]):
            ax_area.plot(np.arange(i+1), data[idx,:i+1, joint_idx])
        ax_area.set_xlabel('Step')
        ax_area.set_title(title)

    def plot_norm_data(ax_area, ylim_range, data, title):
        ax_area.set_ylim(ylim_range)
        ax_area.set_xlim(0, N)
        ax_area.plot(data[idx], linestyle='dashed', c='k')
        for joint_idx in range(data.shape[2]):
            ax_area.plot(np.arange(i+1), data[idx,:i+1, joint_idx])
        ax_area.set_xlabel('Step')
        ax_area.set_title(title)

    plot_data(ax[0,1], (-3.0, 2.0), joints, 'Left Joint angles')
    plot_norm_data(ax[0,2], (0.0, 1.0), norm_joints, 'Normalized left joint angles')
    plot_data(ax[0,3], (-1.0, 1.5), poses, 'Left Poses')
    plot_norm_data(ax[0,4], (0.0, 1.0), norm_poses, 'Normalized left poses')
    plot_data(ax[1,1], (0.0, 2.0), forces, 'Left Forces')
    plot_norm_data(ax[1,2], (0.0, 1.0), norm_forces, 'Normalized left forces')
    plot_data(ax[1,3], (0.0, 1.0), proximities, 'Left Proximities')
    plot_norm_data(ax[1,4], (0.0, 1.0), norm_proximities, 'Normalized left proximities')

ani = anim.FuncAnimation(fig, anim_update, interval=int(N / 10), frames=N)
output_dir = root_dir + "output"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
ani.save(root_dir + "output/check_data_{}.gif".format(idx))
