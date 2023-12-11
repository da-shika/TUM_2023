import os
import argparse
import numpy as np
import matplotlib.pylab as plt
import matplotlib.animation as anim

parser = argparse.ArgumentParser()
parser.add_argument("--dir", type=str, default=None)
parser.add_argument("--file", type=str, default=None)
args = parser.parse_args()

if args.file is None:
    file = os.listdir(args.dir)[0]
else:
    file = args.file
npz_data = np.load(args.dir +"/"+ file)

left_image = npz_data["left_image"]
right_image = npz_data["right_image"]
left_image_dec = npz_data["left_image_dec"]
right_image_dec = npz_data["right_image_dec"]
left_enc_pts = npz_data["left_enc_pts"]
right_enc_pts = npz_data["right_enc_pts"]
left_dec_pts = npz_data["left_dec_pts"]
right_dec_pts = npz_data["right_dec_pts"]

prev_left_pose = npz_data["prev_left_pose"]
prev_right_pose = npz_data["prev_right_pose"]
prev_left_force = npz_data["prev_left_force"]
prev_right_force = npz_data["prev_right_force"]
prev_left_prox = npz_data["prev_left_prox"]
prev_right_prox = npz_data["prev_right_prox"]

real_left_pose = npz_data["real_left_pose"]
real_right_pose = npz_data["real_right_pose"]
real_left_force = npz_data["real_left_force"].reshape((len(npz_data["real_left_force"]), -1))
real_right_force = npz_data["real_right_force"].reshape((len(npz_data["real_right_force"]), -1))
real_left_prox = npz_data["real_left_prox"].reshape((len(npz_data["real_left_prox"]), -1))
real_right_prox = npz_data["real_right_prox"].reshape((len(npz_data["real_right_prox"]), -1))

modality_att = npz_data["modality_att"]
left_att_enc, left_att_force, left_att_prox = np.split(modality_att[0], 3, axis=1)
right_att_enc, right_att_force, right_att_prox = np.split(modality_att[1], 3, axis=1)

T = len(left_image)
fig, ax = plt.subplots(2, 6, figsize=(30, 10), dpi=60)

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

    for j in range(2):
        for k in range(6):
            ax[j,k].cla()

    # plot left camera image
    ax[0,0].imshow(left_image[i, :, :, ::-1])
    for j in range(left_enc_pts.shape[1]):
        ax[0,0].plot(left_enc_pts[i, j, 0], left_enc_pts[i, j, 1], "bo", markersize=6)  # encoder
        ax[0,0].plot(left_dec_pts[i, j, 0], left_dec_pts[i, j, 1], "rx", markersize=6, markeredgewidth=2)  # decoder
    ax[0,0].axis("off")
    ax[0,0].set_title("Input image")

    # plot right camera image
    ax[0,1].imshow(right_image[i, :, :, ::-1])
    for j in range(right_enc_pts.shape[1]):
        ax[0,1].plot(right_enc_pts[i, j, 0], right_enc_pts[i, j, 1], "bo", markersize=6)  # encoder
        ax[0,1].plot(right_dec_pts[i, j, 0], right_dec_pts[i, j, 1], "rx", markersize=6, markeredgewidth=2)  # decoder
    ax[0,1].axis("off")
    ax[0,1].set_title("Input image")

    # plot predicted image
    ax[0,2].imshow(left_image_dec[i, :, :, ::-1])
    ax[0,2].axis("off")
    ax[0,2].set_title("Predicted left image")

    ax[0,3].imshow(right_image_dec[i, :, :, ::-1])
    ax[0,3].axis("off")
    ax[0,3].set_title("Predicted right image")

    def plot_data(prev_data, real_data, ax_area, ylim_range, title):
        ax_area.set_ylim(ylim_range)
        ax_area.set_xlim(0, T)
        ax_area.plot(real_data[1:], linestyle="dashed", c="k")
        for data_idx in range(prev_data.shape[1]):
            ax_area.plot(np.arange(i + 1), prev_data[: i + 1, data_idx])
        ax_area.set_xlabel("Step")
        ax_area.set_title(title)

    def plot_attention(att_enc, att_force, att_prox, ax_area, ylim_range, side):
        att_sum = att_enc + att_force + att_prox
        ax_area.set_ylim(ylim_range)
        ax_area.set_xlim(0, T)
        ax_area.plot(att_enc/att_sum.reshape(T,-1)[1:], linestyle="dashed", c="k")
        ax_area.plot(att_force/att_sum.reshape(T,-1)[1:], linestyle="dashed", c="k")
        ax_area.plot(att_prox/att_sum.reshape(T,-1)[1:], linestyle="dashed", c="k")
        ax_area.plot(np.arange(i + 1), att_enc/att_sum.reshape(T,-1)[: i + 1])      # blue
        ax_area.plot(np.arange(i + 1), att_force/att_sum.reshape(T,-1)[: i + 1])    # orange
        ax_area.plot(np.arange(i + 1), att_prox/att_sum.reshape(T,-1)[: i + 1])     # green
        ax_area.set_xlabel("Step")
        ax_area.set_title(side + " modality attention")

    plot_attention(left_att_enc, left_att_force, left_att_prox, ax[0,4], (0.0, 1.0), "Left")
    plot_attention(right_att_enc, right_att_force, right_att_prox, ax[0,5], (0.0, 1.0), "Right")
    plot_data(prev_left_pose, real_left_pose, ax[1,0], (-1.0, 1.5), "Left pose")
    plot_data(prev_right_pose, real_right_pose, ax[1,1], (-1.0, 1.5), "Right pose")
    plot_data(prev_left_force, real_left_force, ax[1,2], (0.0, 3.0), "Left force")
    plot_data(prev_right_force, real_right_force, ax[1,3], (0.0, 3.0), "Right force")
    plot_data(prev_left_prox, real_left_prox, ax[1,4], (0.0, 1.0), "Left prox")
    plot_data(prev_right_prox, real_right_prox, ax[1,5], (0.0, 1.0), "Right prox")

ani = anim.FuncAnimation(fig, anim_update, interval=int(np.ceil(T / 10)), frames=T)
base_name, _ = os.path.splitext(file)
ani.save(args.dir + "/MTRNN_{}.gif".format(base_name))
# If an error occurs in generating the gif animation or mp4, change the writer (imagemagick/ffmpeg).
#ani.save(save_dir + "/SARNN_{}.gif".format(base_name), writer="imagemagick")
#ani.save(save_dir + "/SARNN_{}.gif".format(base_name), writer="ffmpeg")