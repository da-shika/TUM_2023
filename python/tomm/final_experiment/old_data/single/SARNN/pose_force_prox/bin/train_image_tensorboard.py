#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import os
import sys
import cv2
from PIL import Image
import torch
import argparse
import numpy as np
from tqdm import tqdm
import torch.optim as optim
from collections import OrderedDict
from torch.utils.tensorboard import SummaryWriter
from eipl.utils import EarlyStopping, check_args, set_logdir, normalization, resize_img, tensor2numpy, deprocess_img

try:
    from libs.fullBPTT import fullBPTTtrainer
    from libs.dataset import MultiEpochsDataLoader, MultimodalDataset
    from libs.model import SARNN
except:
    sys.path.append("./libs/")
    from fullBPTT import fullBPTTtrainer
    from dataset import MultiEpochsDataLoader, MultimodalDataset
    from model import SARNN


# argument parser
parser = argparse.ArgumentParser(
    description="Learning spatial autoencoder with recurrent neural network"
)
parser.add_argument("--model", type=str, default="sarnn")
parser.add_argument("--epoch", type=int, default=1000)
parser.add_argument("--batch_size", type=int, default=1)
parser.add_argument("--rec_dim", type=int, default=50)
parser.add_argument("--k_dim", type=int, default=5)     #5 or 10
parser.add_argument("--img_loss", type=float, default=0.1)
parser.add_argument("--joint_loss", type=float, default=1.0)
parser.add_argument("--pt_loss", type=float, default=0.1)
parser.add_argument("--force_loss", type=float, default=0.1)

parser.add_argument("--heatmap_size", type=float, default=0.1)
parser.add_argument("--temperature", type=float, default=1e-4)
parser.add_argument("--stdev", type=float, default=0.1)
parser.add_argument("--lr", type=float, default=1e-3)
parser.add_argument("--optimizer", type=str, default="adam")
parser.add_argument("--log_dir", default="log")
parser.add_argument("--vmin", type=float, default=0.0)
parser.add_argument("--vmax", type=float, default=1.0)
parser.add_argument("--device", type=int, default=0)
parser.add_argument("--n_worker", type=int, default=8)
parser.add_argument("--compile", action="store_true")
parser.add_argument("--CUDA_VISIBLE_DEVICES", type=str)
parser.add_argument("--tag", help="Tag name for snap/log sub directory")
args = parser.parse_args()

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = args.CUDA_VISIBLE_DEVICES

# check args
args = check_args(args)

# calculate the noise level (variance) from the normalized range
stdev = args.stdev * (args.vmax - args.vmin)

# set device id
if args.device >= 0:
    device = "cuda:{}".format(args.device)
else:
    device = "cpu"

# load dataset
detaset_folder = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/re_no_rotate/data/single" + "/"

def load_data(train='train', data_type='left'):
    # load dataset
    minmax = [args.vmin, args.vmax]
    images = np.load(detaset_folder + "{}/images.npy".format(train))
    images = resize_img(images, (64, 64))
    images = images.transpose(0, 1, 4, 2, 3)
    images = normalization(images, (0, 255), minmax)
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
    return images, poses, forces, proximities

# train data
image, left_pose, left_force, left_proximity = load_data(train='train', data_type='left')
_, right_pose, right_force, right_proximity = load_data(train='train', data_type='right')
pose = np.concatenate((left_pose, right_pose), axis=2)
force = np.concatenate((left_force, right_force), axis=2)
prox = np.concatenate((left_proximity, right_proximity), axis=2)
train_dataset = MultimodalDataset(image, pose, force, prox, stdev=stdev)
train_loader = MultiEpochsDataLoader(
    train_dataset,
    batch_size=args.batch_size,
    shuffle=True,
    num_workers=args.n_worker,
    drop_last=False,
    pin_memory=True,
)
print("---train---")
print("image: ", image.shape, image.min(), image.max())
print("poses: ", pose.shape, pose.min(), pose.max())
print("force: ", force.shape, force.min(), force.max())
print("proximity: ", prox.shape, prox.min(), prox.max())


# test data
image, left_pose, left_force, left_proximity = load_data(train='train', data_type='left')
_, right_pose, right_force, right_proximity = load_data(train='train', data_type='right')
pose = np.concatenate((left_pose, right_pose), axis=2)
force = np.concatenate((left_force, right_force), axis=2)
prox = np.concatenate((left_proximity, right_proximity), axis=2)
test_dataset = MultimodalDataset(image, pose, force, prox, stdev=None)
test_loader = MultiEpochsDataLoader(
    test_dataset,
    batch_size=args.batch_size,
    shuffle=True,
    num_workers=args.n_worker,
    drop_last=False,
    pin_memory=True,
)
print("---test---")
print("image: ", image.shape, image.min(), image.max())
print("poses: ", pose.shape, pose.min(), pose.max())
print("force: ", force.shape, force.min(), force.max())
print("proximity: ", prox.shape, prox.min(), prox.max())


# define model
model = SARNN(
    rec_dim=args.rec_dim,
    pose_dim=6,
    force_dim=2,
    prox_dim=2,
    k_dim=args.k_dim,
    heatmap_size=args.heatmap_size,
    temperature=args.temperature,
    im_size=[64, 64]
)

# torch.compile makes PyTorch code run faster
if args.compile:
    torch.set_float32_matmul_precision("high")
    model = torch.compile(model)

# set optimizer
if args.optimizer.casefold() == "adam":
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
elif args.optimizer.casefold() == "radam":
    optimizer = optim.RAdam(model.parameters(), lr=args.lr)
else:
    assert False, "Unknown optimizer name {}. please set Adam or RAdam.".format(
        args.optimizer
    )

# load trainer/tester class
loss_weights = [args.img_loss, args.joint_loss, args.pt_loss, args.force_loss]
trainer = fullBPTTtrainer(model, optimizer, loss_weights=loss_weights, device=device)

### training main
log_dir_path = set_logdir("./" + args.log_dir, args.tag)
save_name = os.path.join(log_dir_path, "SARNN.pth")
writer = SummaryWriter(log_dir=log_dir_path, flush_secs=30)
early_stop = EarlyStopping(patience=1000)
epochs_counter = 0

with tqdm(range(args.epoch)) as pbar_epoch:
    for epoch in pbar_epoch:
        image_list = []
        model.to(device)
        # train and test
        train_loss = trainer.process_epoch(train_loader)
        with torch.no_grad():
            test_loss = trainer.process_epoch(test_loader, training=False)
        writer.add_scalar("Loss/train_loss", train_loss, epoch)
        writer.add_scalar("Loss/test_loss", test_loss, epoch)

        if epochs_counter % 1 == 0:
            with torch.no_grad():
                model.to("cpu")
                model.eval()
                state = None

                def split_key_points(points, image_size=64):
                    _points = tensor2numpy(points)
                    _points = _points.reshape(-1, args.k_dim, 2) * image_size
                    _points = np.clip(_points[0], 0, image_size)
                    return _points
                def denorm_image(image):
                    _image = tensor2numpy(image)
                    _image = np.squeeze(_image, axis=0)
                    _image = deprocess_img(_image, args.vmin, args.vmax)
                    _image = _image.transpose(1, 2, 0)
                    return _image
                def write_gif(log_dir_path, save_name, image_list):
                    gif_path = os.path.join(log_dir_path, save_name)
                    image = []
                    for i in range(len(image_list)):
                        image.append(Image.fromarray(image_list[i]))
                    image[0].save(gif_path, save_all=True, append_images=image[1:], optimize=False, loop=0, duration=60)
                
                for step in range(len(test_dataset[0][0][0])):
                    image = test_dataset[0][0][0][step].unsqueeze(0)
                    pose = test_dataset[0][0][1][step].unsqueeze(0)
                    force = test_dataset[0][0][2][step].unsqueeze(0)
                    prox = test_dataset[0][0][3][step].unsqueeze(0)
                    y_image, _, _, _, enc_pts, dec_pts, state = model(image, pose, force, prox, state)

                image = denorm_image(image)
                enc_pts = split_key_points(enc_pts[0])
                dec_pts = split_key_points(dec_pts[0])
                
                for i in range(args.k_dim):
                    cv2.circle(image, tuple(enc_pts[i].astype(np.int32)), 1, (0,0,255), 1)      # RED
                    cv2.circle(image, tuple(dec_pts[i].astype(np.int32)), 1, (0,255,0), 1)      # GREEN
                
                image_list.append(image[:,:,::-1])

            write_gif(log_dir_path, "image_{}.gif".format(epochs_counter), image_list)        
        
        epochs_counter += 1

        # early stop
        save_ckpt, _ = early_stop(test_loss)

        if save_ckpt:
            trainer.save(epoch, [train_loss, test_loss], save_name)

        # print process bar
        pbar_epoch.set_postfix(OrderedDict(train_loss=train_loss, test_loss=test_loss))
