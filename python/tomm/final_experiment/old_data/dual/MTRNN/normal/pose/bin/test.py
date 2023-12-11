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
from eipl.utils import normalization, resize_img
from eipl.utils import restore_args, tensor2numpy, deprocess_img

try:
    from libs.model import MTRNN
except:
    sys.path.append("./libs/")
    from model import MTRNN

# argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--filename", type=str, default=None)
parser.add_argument("--idx", type=str, default="0")
parser.add_argument("--input_param", type=float, default=1.0)
parser.add_argument("--pretrained", action="store_true")
args = parser.parse_args()

# check args
assert args.filename or args.pretrained, "Please set filename or pretrained"

# restore parameters
dir_name = os.path.split(args.filename)[0]
params = restore_args(os.path.join(dir_name, "args.json"))
idx = int(args.idx)

# load dataset
detaset_folder = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/re_no_rotate/data/dual" + "/"
minmax = [params["vmin"], params["vmax"]]

left_pose_bounds = np.load(detaset_folder + "left_pose_bounds.npy")
right_pose_bounds = np.load(detaset_folder + "right_pose_bounds.npy")

def load_data(train="train", data_type="left"):
    # load dataset
    images = np.load(detaset_folder + "{}/{}_images.npy".format(train, data_type))
    images = resize_img(images, (64, 64))
    images = images.transpose(0, 1, 4, 2, 3)
    images = normalization(images, (0, 255), minmax)
    poses = np.load(detaset_folder + "{}/{}_poses.npy".format(train, data_type))
    pose_bounds = np.load(detaset_folder +  "{}_pose_bounds.npy".format(data_type))
    poses = normalization(poses, pose_bounds, minmax)
    return images, poses

left_image, left_pose = load_data(train='test', data_type='left')
right_image, right_pose = load_data(train='test', data_type='right')

left_image = left_image[idx]
right_image = right_image[idx]
left_pose = left_pose[idx]

print("left_image: ", left_image.shape, left_image.min(), left_image.max())
print("left_pose: ", left_pose.shape, left_pose.min(), left_pose.max())
print("right_image: ", right_image.shape, right_image.min(), right_image.max())
print("right_pose: ", right_pose.shape, right_pose.min(), right_pose.max())

# define model
model = MTRNN(
    fast_dim=params["fast_dim"], slow_dim=params["slow_dim"], union_dim=params["union_dim"],
    fast_tau=params["fast_tau"], slow_tau=params["slow_tau"], union_tau=params["union_tau"],
    pose_dim=3,
    k_dim=params["k_dim"],
    heatmap_size=params["heatmap_size"],
    temperature=params["temperature"],
    im_size=[64, 64],
)

if params["compile"]:
    model = torch.compile(model)

# load weight
ckpt = torch.load(args.filename, map_location=torch.device("cpu"))
model.load_state_dict(ckpt["model_state_dict"])
model.eval()

# Inference
img_size = 64
image_list, pose_list = [], []
left_enc_pts_list, left_dec_pts_list = [], []
right_enc_pts_list, right_dec_pts_list = [], []
state = [None, None, None]
nloop = len(left_image)

for loop_ct in range(nloop):
    # load data and normalization
    img_t = [torch.Tensor(left_image[loop_ct : loop_ct + 1]),
            torch.Tensor(right_image[loop_ct : loop_ct + 1]),]
    pose_t = [torch.Tensor(left_pose[loop_ct : loop_ct + 1]),
            torch.Tensor(right_pose[loop_ct : loop_ct + 1]),]

    # closed loop
    """if loop_ct > 0:
        img_t = args.input_param * img_t + (1.0 - args.input_param) * y_image
        pose_t = args.input_param * pose_t + (1.0 - args.input_param) * y_pose
        force_t = args.input_param * force_t + (1.0 - args.input_param) * y_force
        proximity_t = args.input_param * proximity_t + (1.0 - args.input_param) * y_proximity"""

    # predict rnn
    y_image, y_pose, ect_pts, dec_pts, state = model(img_t, pose_t, state)

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
    
    pred_pose = de_norm(y_pose, minmax, left_pose_bounds, right_pose_bounds)

    # append data
    image_list.append(pred_image)
    pose_list.append(pred_pose)
    left_enc_pts_list.append(tensor2numpy(ect_pts[0][0]))
    left_dec_pts_list.append(tensor2numpy(dec_pts[0][0]))
    right_enc_pts_list.append(tensor2numpy(ect_pts[1][0]))
    right_dec_pts_list.append(tensor2numpy(dec_pts[1][0]))

    print("loop_ct:{}, pose:{}".format(loop_ct, pose_list))

pred_image = np.array(image_list)
pred_pose = np.array(pose_list)

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
fig, ax = plt.subplots(2, 3, figsize=(15, 10), dpi=60)

bbox = ax[1,0].get_position()
ax[1,0].set_position(Bbox.from_bounds(bbox.x0, bbox.y0, bbox.width*2, bbox.height))

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

    _remove_axis(ax[1,1])
    _remove_axis(ax[1,2])

    for j in range(2):
        for k in range(3):
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
    ax[1,0].imshow(pred_image[i, :, :, ::-1])
    ax[1,0].axis("off")
    ax[1,0].set_title("Predicted image")

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

    plot_data(left_pose, left_pose_bounds, right_pose, right_pose_bounds, pred_pose, \
              ax[0,2], (-1.0, 1.5), "Pose")


ani = anim.FuncAnimation(fig, anim_update, interval=int(np.ceil(T / 10)), frames=T)
save_dir = "./output/" + dir_name[6:]
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
ani.save(save_dir + "/MTRNN_{}_{}_{}.gif".format(params["tag"], idx, args.input_param))
# If an error occurs in generating the gif animation or mp4, change the writer (imagemagick/ffmpeg).
# ani.save("./output/PCA_MTRNN_{}.gif".format(params["tag"]), writer="imagemagick")
# ani.save("./output/PCA_MTRNN_{}.mp4".format(params["tag"]), writer="ffmpeg")
