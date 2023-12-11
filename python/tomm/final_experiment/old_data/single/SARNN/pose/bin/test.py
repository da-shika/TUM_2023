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
    from libs.model import SARNN
except:
    sys.path.append("./libs/")
    from model import SARNN


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
detaset_folder = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/re_no_rotate/data/single/"
minmax = [params["vmin"], params["vmax"]]

left_pose_bounds = np.load(detaset_folder + "left_pose_bounds.npy")
right_pose_bounds = np.load(detaset_folder + "right_pose_bounds.npy")

def load_data(train="train", data_type="left"):
    # load dataset
    images = np.load(detaset_folder + "{}/images.npy".format(train))
    images = resize_img(images, (64, 64))
    images = images.transpose(0, 1, 4, 2, 3)
    images = normalization(images, (0, 255), minmax)
    poses = np.load(detaset_folder + "{}/{}_poses.npy".format(train, data_type))
    pose_bounds = np.load(detaset_folder +  "{}_pose_bounds.npy".format(data_type))
    poses = normalization(poses, pose_bounds, minmax)
    return images, poses

# test data
image, left_pose = load_data(train='test', data_type='left')
_, right_pose = load_data(train='test', data_type='right')
pose = np.concatenate((left_pose, right_pose), axis=2)

image = image[idx]
pose = pose[idx]
left_pose = left_pose[idx]
right_pose = right_pose[idx]

print("---test---")
print("image: ", image.shape, image.min(), image.max())
print("poses: ", pose.shape, pose.min(), pose.max())


# define model
model = SARNN(
    rec_dim=params["rec_dim"],
    pose_dim=6,
    k_dim=params["k_dim"],
    heatmap_size=params["heatmap_size"],
    temperature=params["temperature"],
    im_size=[64, 64]
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
ect_pts_list, dec_pts_list = [], []
state = None
nloop = len(image)
for loop_ct in range(nloop):
    # load data and normalization
    img_t = torch.Tensor(image[loop_ct : loop_ct + 1])
    pose_t = torch.Tensor(pose[loop_ct : loop_ct + 1])

    """# closed loop
    if loop_ct > 0:
        img_t = args.input_param * img_t + (1.0 - args.input_param) * y_image
        joint_t = args.input_param * joint_t + (1.0 - args.input_param) * y_joint"""

    # predict rnn
    y_image, y_pose, ect_pts, dec_pts, state = model(img_t, pose_t, state)

    # denormalization
    pred_image = tensor2numpy(y_image[0])
    pred_image = deprocess_img(pred_image, params["vmin"], params["vmax"])
    pred_image = pred_image.transpose(1, 2, 0)

    def de_norm(y, minmax, l_bounds, r_bounds):
        _pred = tensor2numpy(y[0])
        _l_pred = _pred[: int(len(_pred)/2)]
        l_pred = normalization(_l_pred, minmax, l_bounds)
        _r_pred = _pred[int(len(_pred)/2) :]
        r_pred = normalization(_r_pred, minmax, r_bounds)
        pred = np.concatenate((l_pred, r_pred), axis=0)
        return pred
    
    pred_pose = de_norm(y_pose, minmax, left_pose_bounds, right_pose_bounds)

    # send pred_joint to robot
    # send_command(pred_joint)
    # pub.publish(pred_joint)

    # append data
    image_list.append(pred_image)
    pose_list.append(pred_pose)
    ect_pts_list.append(tensor2numpy(ect_pts[0]))
    dec_pts_list.append(tensor2numpy(dec_pts[0]))

    print("loop_ct:{}, joint:{}".format(loop_ct, pose_list))

pred_image = np.array(image_list)
pred_pose = np.array(pose_list)

# split key points
ect_pts = np.array(ect_pts_list)
dec_pts = np.array(dec_pts_list)
ect_pts = ect_pts.reshape(-1, params["k_dim"], 2) * img_size
dec_pts = dec_pts.reshape(-1, params["k_dim"], 2) * img_size
enc_pts = np.clip(ect_pts, 0, img_size)
dec_pts = np.clip(dec_pts, 0, img_size)


# plot images
T = len(image)
fig, ax = plt.subplots(2, 2, figsize=(10, 10), dpi=60)


def anim_update(i):
    for j in range(2):
        for k in range(2):
            ax[j,k].cla()

    # plot camera image
    _image = image.transpose(0,2,3,1)
    ax[0,0].imshow(_image[i, :, :, ::-1])
    for j in range(params["k_dim"]):
        ax[0,0].plot(ect_pts[i, j, 0], ect_pts[i, j, 1], "bo", markersize=6)  # encoder
        ax[0,0].plot(dec_pts[i, j, 0], dec_pts[i, j, 1], "rx", markersize=6, markeredgewidth=2)  # decoder
    ax[0,0].axis("off")
    ax[0,0].set_title("Input image")

    # plot predicted image
    ax[0,1].imshow(pred_image[i, :, :, ::-1])
    ax[0,1].axis("off")
    ax[0,1].set_title("Predicted image")

    def plot_data(left_data, left_bounds, right_data, right_bounds, pred_data, ax_area, ylim_range, title):
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

    plot_data(left_pose, left_pose_bounds, right_pose, right_pose_bounds, pred_pose, ax[1,0], (-1.0, 1.5), "Pose")


ani = anim.FuncAnimation(fig, anim_update, interval=int(np.ceil(T / 10)), frames=T)
save_dir = "./output/" + dir_name[6:]
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
ani.save(save_dir + "/SARNN_{}_{}_{}.gif".format(params["tag"], idx, args.input_param))

# If an error occurs in generating the gif animation or mp4, change the writer (imagemagick/ffmpeg).
# ani.save("./output/PCA_SARNN_{}.gif".format(params["tag"]), writer="imagemagick")
# ani.save("./output/PCA_SARNN_{}.mp4".format(params["tag"]), writer="ffmpeg")
