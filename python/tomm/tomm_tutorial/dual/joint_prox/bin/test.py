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
from matplotlib.transforms import Bbox
import matplotlib.animation as anim
from eipl.data import SampleDownloader, WeightDownloader
from eipl.utils import normalization, resize_img
from eipl.utils import restore_args, tensor2numpy, deprocess_img

try:
    from libs.model import SARNN
except:
    sys.path.append("./libs/")
    from model import SARNN

# argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--filename", type=str, default=None)
parser.add_argument("--idx", type=str, default="0")
parser.add_argument("--input_param", type=float, default=1.0)
args = parser.parse_args()

# check args
assert args.filename or args.pretrained, "Please set filename or pretrained"


# restore parameters
dir_name = os.path.split(args.filename)[0]
params = restore_args(os.path.join(dir_name, "args.json"))
idx = int(args.idx)

detaset_folder = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/data/dual" + "/"

minmax = [params["vmin"], params["vmax"]]
left_joint_bounds = np.load(detaset_folder + "left_joint_bounds.npy")
right_joint_bounds = np.load(detaset_folder + "right_joint_bounds.npy")
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

    # skin sensor
    proximities = np.load(detaset_folder + "{}/{}_proximities.npy".format(train, data_type))
    proximitiy_bounds = np.load(detaset_folder + "{}_proximity_bounds.npy".format(data_type))
    proximities = normalization(proximities, proximitiy_bounds, minmax)
    return images, joints, proximities


# load dataset
left_image, left_joint, left_proximity = load_data(train='test', data_type='left')
right_image, right_joint, right_proximity = load_data(train='test', data_type='right')

left_image = left_image[idx]
right_image = right_image[idx]
left_joint = left_joint[idx]
right_joint = right_joint[idx]

left_proximity = left_proximity[idx]
right_proximity = right_proximity[idx]

print("left_image: ", left_image.shape, left_image.min(), left_image.max())
print("left_joint: ", left_joint.shape, left_joint.min(), left_joint.max())
print("left_proximity: ", left_proximity.shape, left_proximity.min(), left_proximity.max())
print("right_image: ", right_image.shape, right_image.min(), right_image.max())
print("right_joint: ", right_joint.shape, right_joint.min(), right_joint.max())
print("right_proximity: ", right_proximity.shape, right_proximity.min(), right_proximity.max())

# define model
model = SARNN(
    rec_dim=params["rec_dim"],
    joint_dim=6,
    proximity_dim=10,
    k_dim=params["k_dim"],
    heatmap_size=params["heatmap_size"],
    temperature=params["temperature"],
    im_size=[64, 64],
)

# load weight
ckpt = torch.load(args.filename, map_location=torch.device("cpu"))
model.load_state_dict(ckpt["model_state_dict"])
model.eval()

# Inference
img_size = 64
image_list, joint_list, proximity_list = [], [], []
left_enc_pts_list, left_dec_pts_list = [], []
right_enc_pts_list, right_dec_pts_list = [], []
state = [None, None, None]
nloop = len(left_image)
for loop_ct in range(nloop):
    # load data and normalization
    img_t = [torch.Tensor(left_image[loop_ct : loop_ct + 1]),
            torch.Tensor(right_image[loop_ct : loop_ct + 1]),]
    joint_t = [torch.Tensor(left_joint[loop_ct : loop_ct + 1]),
            torch.Tensor(right_joint[loop_ct : loop_ct + 1]),]
    
    proximity_t = [torch.Tensor(left_proximity[loop_ct : loop_ct + 1]),
            torch.Tensor(right_proximity[loop_ct : loop_ct + 1]),]

    # predict rnn
    y_image, y_joint, y_proximity, ect_pts, dec_pts, state = model(img_t, joint_t, proximity_t, state)

    # denormalization
    pred_image1 = tensor2numpy(y_image[0][0])
    pred_image1 = deprocess_img(pred_image1, params["vmin"], params["vmax"])
    pred_image1 = pred_image1.transpose(1, 2, 0)
    pred_image2 = tensor2numpy(y_image[1][0])
    pred_image2 = deprocess_img(pred_image2, params["vmin"], params["vmax"])
    pred_image2 = pred_image2.transpose(1, 2, 0)
    pred_image = np.concatenate((pred_image1, pred_image2), axis=1)

    def de_norm(y, minmax, l_bounds, r_bounds):
        _p_1 = tensor2numpy(y[0][0])
        p_1 = normalization(_p_1, minmax, l_bounds)
        _p_2 = tensor2numpy(y[1][0])
        p_2 = normalization(_p_2, minmax, r_bounds)
        pred = np.concatenate((p_1, p_2), axis=0)
        return pred
    
    pred_joint = de_norm(y_joint, minmax, left_joint_bounds, right_joint_bounds)
    pred_proximity = de_norm(y_proximity, minmax, left_proximity_bounds, right_proximity_bounds)
    # send pred_joint to robot
    # send_command(pred_joint)
    # pub.publish(pred_joint)

    # append data
    image_list.append(pred_image)
    joint_list.append(pred_joint)
    proximity_list.append(pred_proximity)
    left_enc_pts_list.append(tensor2numpy(ect_pts[0][0]))
    left_dec_pts_list.append(tensor2numpy(dec_pts[0][0]))
    right_enc_pts_list.append(tensor2numpy(ect_pts[1][0]))
    right_dec_pts_list.append(tensor2numpy(dec_pts[1][0]))

    print("loop_ct:{}, joint:{}".format(loop_ct, pred_joint))

pred_image = np.array(image_list)
pred_joint = np.array(joint_list)
pred_proximity = np.array(proximity_list)

# split key points
left_enc_pts = np.array(left_enc_pts_list)
left_dec_pts = np.array(left_dec_pts_list)
left_enc_pts = left_enc_pts.reshape(-1, params["k_dim"], 2) * img_size
left_dec_pts = left_dec_pts.reshape(-1, params["k_dim"], 2) * img_size
left_enc_pts = np.clip(left_enc_pts, 0, img_size)
left_dec_pts = np.clip(left_dec_pts, 0, img_size)

right_enc_pts = np.array(right_enc_pts_list)
right_dec_pts = np.array(right_dec_pts_list)
right_enc_pts = right_enc_pts.reshape(-1, params["k_dim"], 2) * img_size
right_dec_pts = right_dec_pts.reshape(-1, params["k_dim"], 2) * img_size
right_enc_pts = np.clip(right_enc_pts, 0, img_size)
right_dec_pts = np.clip(right_dec_pts, 0, img_size)

# plot images
T = len(left_image)
fig, ax = plt.subplots(2, 4, figsize=(20, 10), dpi=60)

bbox = ax[0,2].get_position()
ax[0,2].set_position(Bbox.from_bounds(bbox.x0, bbox.y0, bbox.width*2, bbox.height))

def anim_update(i):
    def _remove_axis(ax_area):
        ax_area.clear()
        ax_area.spines["top"].set_visible(False)
        ax_area.spines["right"].set_visible(False)
        ax_area.spines["bottom"].set_visible(False)
        ax_area.spines["left"].set_visible(False)
        ax_area.tick_params(axis="both", which="both", left=False, right=False, bottom=False, top=False, \
                            labelbottom=False, labelleft=False, labelright=False, labeltop=False)
        ax_area.set_facecolor("none")

    _remove_axis(ax[0,3])
    _remove_axis(ax[1,2])
    _remove_axis(ax[1,3])

    for j in range(2):
        for k in range(4):
            ax[j,k].cla()

    # plot left camera image
    _left_image = left_image.transpose(0,2,3,1)
    _right_image = right_image.transpose(0,2,3,1)
    ax[0,0].imshow(_left_image[i, :, :, ::-1])
    for j in range(params["k_dim"]):
        ax[0,0].plot(left_enc_pts[i, j, 0], left_enc_pts[i, j, 1], "bo", markersize=6)  # encoder
        ax[0,0].plot(
            left_dec_pts[i, j, 0], left_dec_pts[i, j, 1], "rx", markersize=6, markeredgewidth=2
        )  # decoder
    ax[0,0].axis("off")
    ax[0,0].set_title("Input image")

    # plot right camera image
    ax[0,1].imshow(_right_image[i, :, :, ::-1])
    for j in range(params["k_dim"]):
        ax[0,1].plot(right_enc_pts[i, j, 0], right_enc_pts[i, j, 1], "bo", markersize=6)  # encoder
        ax[0,1].plot(
            right_dec_pts[i, j, 0], right_dec_pts[i, j, 1], "rx", markersize=6, markeredgewidth=2
        )  # decoder
    ax[0,1].axis("off")
    ax[0,1].set_title("Input image")

    # plot predicted image
    ax[0,2].imshow(pred_image[i, :, :, ::-1])
    ax[0,2].axis("off")
    ax[0,2].set_title("Predicted image")

    def plot_data(left_data, left_bounds, right_data, right_bounds, pred_data, \
                  ax_area, ylim_range, title):
        _left_data = normalization(left_data, minmax, left_bounds)
        _right_data = normalization(right_data, minmax, right_bounds)
        ax_area.set_ylim(ylim_range)
        ax_area.set_xlim(0, T)
        ax_area.plot(_left_data[1:], linestyle="dashed", c="k")
        ax_area.plot(_right_data[1:], linestyle="dashed", c="k")
        for data_idx in range(left_data.shape[1] * 2):
            ax_area.plot(np.arange(i + 1), pred_data[: i + 1, data_idx])
        ax_area.set_xlabel("Step")
        ax_area.set_title(title)

    plot_data(left_joint, left_joint_bounds, right_joint, right_joint_bounds, pred_joint, \
              ax[1,0], (-3.0, 2.0), "Joint angles")
    plot_data(left_proximity, left_proximity_bounds, right_proximity, right_proximity_bounds, pred_proximity, \
              ax[1,1], (0.0, 1.0), "Proximity")


ani = anim.FuncAnimation(fig, anim_update, interval=int(np.ceil(T / 10)), frames=T)
save_dir = "./output/" + dir_name[6:]
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
ani.save(save_dir + "/SARNN_{}_{}_{}.gif".format(params["tag"], idx, args.input_param))