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
root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/data/dual" + "/"

left_joints = np.load(root_dir + "test/left_joints.npy")
right_joints = np.load(root_dir + "test/right_joints.npy")
left_joint_bounds = np.load(root_dir + "left_joint_bounds.npy")
right_joint_bounds = np.load(root_dir + "right_joint_bounds.npy")

left_poses = np.load(root_dir + "test/left_poses.npy")
right_poses = np.load(root_dir + "test/right_poses.npy")
left_pose_bounds = np.load(root_dir + "left_pose_bounds.npy")
right_pose_bounds = np.load(root_dir + "right_pose_bounds.npy")

left_images = np.load(root_dir + "test/left_images.npy")
right_images = np.load(root_dir + "test/right_images.npy")

left_forces = np.load(root_dir + "test/left_forces.npy")
right_forces = np.load(root_dir + "test/right_forces.npy")
left_force_bounds = np.load(root_dir + "left_force_bounds.npy")
right_force_bounds = np.load(root_dir + "right_force_bounds.npy")

left_proximities = np.load(root_dir + "test/left_proximities.npy")
right_proximities = np.load(root_dir + "test/right_proximities.npy")
left_proximity_bounds = np.load(root_dir + "left_proximity_bounds.npy")
right_proximity_bounds = np.load(root_dir + "right_proximity_bounds.npy")

N = left_images.shape[1]


# normalized
minmax = [0.1, 0.9]
norm_left_joints = normalization( left_joints, left_joint_bounds, minmax )
norm_right_joints = normalization( right_joints, right_joint_bounds, minmax )
norm_left_poses = normalization( left_poses, left_pose_bounds, minmax )
norm_right_poses = normalization( right_poses, right_pose_bounds, minmax )
norm_left_forces = normalization( left_forces, left_force_bounds, minmax )
norm_right_forces = normalization( right_forces, right_force_bounds, minmax )
norm_left_proximities = normalization( left_proximities, left_proximity_bounds, minmax )
norm_right_proximities = normalization( right_proximities, right_proximity_bounds, minmax )

# print data information
print('load test data, index number is {}'.format(idx))
print('Left Joint: shape={}, min={:.3g}, max={:.3g}'.format(left_joints.shape, left_joints.min(), left_joints.max()) )
print('Right Joint: shape={}, min={:.3g}, max={:.3g}'.format(right_joints.shape, right_joints.min(), right_joints.max()) )
print('Left Norm Joint: shape={}, min={:.3g}, max={:.3g}'.format(norm_left_joints.shape, norm_left_joints.min(), norm_left_joints.max()) )
print('Right Norm Joint: shape={}, min={:.3g}, max={:.3g} \n'.format(norm_right_joints.shape, norm_right_joints.min(), norm_right_joints.max()) )

print('Left Pose: shape={}, min={:.3g}, max={:.3g}'.format(left_poses.shape, left_poses.min(), left_poses.max()) )
print('Right Pose: shape={}, min={:.3g}, max={:.3g}'.format(right_poses.shape, right_poses.min(), right_poses.max()) )
print('Left Norm Pose: shape={}, min={:.3g}, max={:.3g}'.format(norm_left_poses.shape, norm_left_poses.min(), norm_left_poses.max()) )
print('Right Norm Pose: shape={}, min={:.3g}, max={:.3g} \n'.format(norm_right_poses.shape, norm_right_poses.min(), norm_right_poses.max()) )

print('Left Force: shape={}, min={:.3g}, max={:.3g}'.format(left_forces.shape, left_forces.min(), left_forces.max()) )
print('Right Force: shape={}, min={:.3g}, max={:.3g}'.format(right_forces.shape, right_forces.min(), right_forces.max()) )
print('Left Norm Force: shape={}, min={:.3g}, max={:.3g}'.format(norm_left_forces.shape, norm_left_forces.min(), norm_left_forces.max()) )
print('Right Norm Force: shape={}, min={:.3g}, max={:.3g} \n'.format(norm_right_forces.shape, norm_right_forces.min(), norm_right_forces.max()) )

print('Left Proximity: shape={}, min={:.3g}, max={:.3g}'.format(left_proximities.shape, left_proximities.min(), left_proximities.max()) )
print('Right Proximity: shape={}, min={:.3g}, max={:.3g}'.format(right_proximities.shape, right_proximities.min(), right_proximities.max()) )
print('Left Norm Proximity: shape={}, min={:.3g}, max={:.3g}'.format(norm_left_proximities.shape, norm_left_proximities.min(), norm_left_proximities.max()) )
print('Right Norm Proximity: shape={}, min={:.3g}, max={:.3g} \n'.format(norm_right_proximities.shape, norm_right_proximities.min(), norm_right_proximities.max()) )

print('Left Image: shape={}'.format(left_images.shape) )
print('Right Image: shape={}'.format(right_images.shape) )


# plot images and normalized joints
fig, ax = plt.subplots(2, 9, figsize=(45, 10), dpi=60)

def anim_update(i):
    for j in range(2):
        for k in range(9):
            ax[j,k].cla()

    # plot left image
    ax[0,0].imshow(left_images[idx,i,:,:,::-1])
    ax[0,0].axis('off')
    ax[0,0].set_title('Left image')

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

    plot_data(ax[0,1], (-3.0, 2.0), left_joints, 'Left Joint angles')
    plot_norm_data(ax[0,2], (0.0, 1.0), norm_left_joints, 'Normalized left joint angles')
    plot_data(ax[0,3], (-1.0, 1.5), left_poses, 'Left Poses')
    plot_norm_data(ax[0,4], (0.0, 1.0), norm_left_poses, 'Normalized left poses')
    plot_data(ax[0,5], (0.0, 2.0), left_forces, 'Left Forces')
    plot_norm_data(ax[0,6], (0.0, 1.0), norm_left_forces, 'Normalized left forces')
    plot_data(ax[0,7], (0.0, 1.0), left_proximities, 'Left Proximities')
    plot_norm_data(ax[0,8], (0.0, 1.0), norm_left_proximities, 'Normalized left proximities')


    # plot right image
    ax[1,0].imshow(right_images[idx,i,:,:,::-1])
    ax[1,0].axis('off')
    ax[1,0].set_title('Right image')

    plot_data(ax[1,1], (-3.0, 2.0), right_joints, 'Right Joint angles')
    plot_norm_data(ax[1,2], (0.0, 1.0), norm_right_joints, 'Normalized right joint angles')
    plot_data(ax[1,3], (-1.0, 1.5), right_poses, 'Right Poses')
    plot_norm_data(ax[1,4], (0.0, 1.0), norm_right_poses, 'Normalized right poses')
    plot_data(ax[1,5], (0.0, 4.0), right_forces, 'Right Forces')
    plot_norm_data(ax[1,6], (0.0, 1.0), norm_right_forces, 'Normalized right forces')
    plot_data(ax[1,7], (0.0, 1.0), right_proximities, 'Right Proximities')
    plot_norm_data(ax[1,8], (0.0, 1.0), norm_right_proximities, 'Normalized right proximities')


ani = anim.FuncAnimation(fig, anim_update, interval=int(N / 10), frames=N)
output_dir = root_dir + "output"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
ani.save(root_dir + "output/check_data_{}.gif".format(idx))
