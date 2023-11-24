#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import os
import sys
import torch
import argparse
import numpy as np
import matplotlib.pylab as plt
import matplotlib.animation as anim
from sklearn.decomposition import PCA
from eipl.data import SampleDownloader
from eipl.utils import restore_args, tensor2numpy, normalization, resize_img

try:
    from libs.model import SARNN
except:
    sys.path.append("./libs/")
    from model import SARNN

# argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--filename", type=str, default=None)
args = parser.parse_args()

# check args
assert args.filename, "Please set filename"

# restore parameters
dir_name = os.path.split(args.filename)[0]
params = restore_args(os.path.join(dir_name, "args.json"))

#------------------------------------------------------------------------------------------------------------------
# load dataset
detaset_folder = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/data/dual" + "/"

minmax = [params["vmin"], params["vmax"]]
left_joint_bounds = np.load(detaset_folder + "left_joint_bounds.npy")
right_joint_bounds = np.load(detaset_folder + "right_joint_bounds.npy")
left_pose_bounds = np.load(detaset_folder + "left_pose_bounds.npy")
right_pose_bounds = np.load(detaset_folder + "right_pose_bounds.npy")

left_force_bounds = np.load(detaset_folder + "left_force_bounds.npy")
right_force_bounds = np.load(detaset_folder + "right_force_bounds.npy")
left_proximity_bounds = np.load(detaset_folder + "left_proximity_bounds.npy")
right_proximity_bounds = np.load(detaset_folder + "right_proximity_bounds.npy")

def load_data(train="train", data_type="left"):
    # load dataset
    images = np.load(detaset_folder + "{}/{}_images.npy".format(train, data_type))
    images = resize_img(images, (64, 64))
    images = images.transpose(0, 1, 4, 2, 3)
    images = normalization(images, (0, 255), minmax)
    joints = np.load(detaset_folder + "{}/{}_joints.npy".format(train, data_type))
    joint_bounds = np.load(detaset_folder +  "{}_joint_bounds.npy".format(data_type))
    joints = normalization(joints, joint_bounds, minmax)
    poses = np.load(detaset_folder + "{}/{}_poses.npy".format(train, data_type))
    pose_bounds = np.load(detaset_folder +  "{}_pose_bounds.npy".format(data_type))
    poses = normalization(poses, pose_bounds, minmax)

    # skin sensor
    forces = np.load(detaset_folder + "{}/{}_forces.npy".format(train, data_type))
    force_bounds = np.load(detaset_folder + "{}_force_bounds.npy".format(data_type))
    forces = normalization(forces, force_bounds, minmax)
    proximities = np.load(detaset_folder + "{}/{}_proximities.npy".format(train, data_type))
    proximitiy_bounds = np.load(detaset_folder + "{}_proximity_bounds.npy".format(data_type))
    proximities = normalization(proximities, proximitiy_bounds, minmax)
    return images, joints, poses, forces, proximities

left_image, left_joint, left_pose, left_force, left_proximity = load_data(train='test', data_type='left')
right_image, right_joint, right_pose, right_force, right_proximity = load_data(train='test', data_type='right')

print("left_image: ", left_image.shape, left_image.min(), left_image.max())
print("left_joint: ", left_joint.shape, left_joint.min(), left_joint.max())
print("left_pose: ", left_pose.shape, left_pose.min(), left_pose.max())
print("left_force: ", left_force.shape, left_force.min(), left_force.max())
print("left_proximity: ", left_proximity.shape, left_proximity.min(), left_proximity.max())
print("right_image: ", right_image.shape, right_image.min(), right_image.max())
print("right_joint: ", right_joint.shape, right_joint.min(), right_joint.max())
print("right_pose: ", right_pose.shape, right_pose.min(), right_pose.max())
print("right_force: ", right_force.shape, right_force.min(), right_force.max())
print("right_proximity: ", right_proximity.shape, right_proximity.min(), right_proximity.max())

#------------------------------------------------------------------------------------------------------------------
# define model
model = SARNN(
    rec_dim=params["rec_dim"],
    joint_dim=6,
    pose_dim=7,
    force_dim=10,
    proximity_dim=10,
    k_dim=params["k_dim"],
    heatmap_size=params["heatmap_size"],
    temperature=params["temperature"],
    im_size=[64, 64],
)

ckpt = torch.load(args.filename, map_location=torch.device("cpu"))
model.load_state_dict(ckpt["model_state_dict"])
model.eval()

# Inference
states = [[], [], []]
state = [None, None, None]
nloop = left_image.shape[1]
for loop_ct in range(nloop):
    img_t = [torch.Tensor(left_image[:, loop_ct]), torch.Tensor(right_image[:, loop_ct]),]
    joint_t = [torch.Tensor(left_joint[:, loop_ct]), torch.Tensor(right_joint[:, loop_ct]),]
    pose_t = [torch.Tensor(left_pose[:, loop_ct]), torch.Tensor(right_pose[:, loop_ct]),]
    
    force_t = [torch.Tensor(left_force[:, loop_ct]), torch.Tensor(right_force[:, loop_ct]),]
    proximity_t = [torch.Tensor(left_proximity[:, loop_ct]), torch.Tensor(right_proximity[:, loop_ct]),]

    # predict rnn
    _, _, _, _, _, _, _, state = model(img_t, joint_t, pose_t, force_t, proximity_t, state)
    states[0].append(state[0][0])
    states[1].append(state[1][0])
    states[2].append(state[2][0])


def make_pca(states):
    states = torch.permute(torch.stack(states), (1, 0, 2))
    states = tensor2numpy(states)
    # Reshape the state from [N,T,D] to [-1,D] for PCA of RNN.
    # N is the number of datasets
    # T is the sequence length
    # D is the dimension of the hidden state
    N, T, D = states.shape
    states = states.reshape(-1, D)

    # plot pca
    loop_ct = float(360) / T
    pca_dim = 3
    pca = PCA(n_components=pca_dim).fit(states)
    pca_val = pca.transform(states)
    # Reshape the states from [-1, pca_dim] to [N,T,pca_dim] to
    # visualize each state as a 3D scatter.
    pca_val = pca_val.reshape(N, T, pca_dim)
    
    return T, loop_ct, pca, pca_val

T_0, loop_ct_0, pca_0, pca_val_0 = make_pca(states[0])
T_1, loop_ct_1, pca_1, pca_val_1 = make_pca(states[1])
T_2, loop_ct_2, pca_2, pca_val_2 = make_pca(states[2])
T = max(T_0, T_1, T_2)

fig, ax = plt.subplots(1, 3, figsize=(15, 5), dpi=60, subplot_kw={"projection": "3d"})

def anim_update(i):
    for j in range(3):
        ax[j].cla()

    def _plot_pca(loop_ct, ax_area, pca_val, pca, title):
        angle = int(loop_ct * i)
        ax_area.view_init(30, angle)

        c_list = ["C0", "C1", "C2", "C3", "C4"]
        for n, color in enumerate(c_list):
            ax_area.scatter(pca_val[n, 1:, 0], pca_val[n, 1:, 1], pca_val[n, 1:, 2], color=color, s=3.0)

        ax_area.scatter(pca_val[n, 0, 0], pca_val[n, 0, 1], pca_val[n, 0, 2], color="k", s=30.0)
        pca_ratio = pca.explained_variance_ratio_ * 100
        ax_area.set_xlabel("PC1 ({:.1f}%)".format(pca_ratio[0]))
        ax_area.set_ylabel("PC2 ({:.1f}%)".format(pca_ratio[1]))
        ax_area.set_zlabel("PC3 ({:.1f}%)".format(pca_ratio[2]))
        ax_area.set_title(title)

    _plot_pca(loop_ct_0, ax[0], pca_val_0, pca_0, "Left state")
    _plot_pca(loop_ct_1, ax[1], pca_val_1, pca_1, "Right state")
    _plot_pca(loop_ct_2, ax[2], pca_val_2, pca_2, "Integrate state")

ani = anim.FuncAnimation(fig, anim_update, interval=int(np.ceil(T / 10)), frames=T)
save_dir = "./output/" + dir_name[6:]
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
ani.save(save_dir + "/PCA_SARNN_{}.gif".format(params["tag"]))

# If an error occurs in generating the gif animation, change the writer (imagemagick/ffmpeg).
#ani.save("./output/PCA_SARNN_{}.gif".format(params["tag"]), writer="imagemagick")
#ani.save("./output/PCA_SARNN_{}.gif".format(params["tag"]), writer="ffmpeg")
