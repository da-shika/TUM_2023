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
from tqdm import tqdm
import torch.optim as optim
from collections import OrderedDict
from torch.utils.tensorboard import SummaryWriter
from eipl.utils import EarlyStopping, check_args, set_logdir, normalization, resize_img

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
parser.add_argument("--batch_size", type=int, default=5)
parser.add_argument("--rec_dim", type=int, default=50)
parser.add_argument("--k_dim", type=int, default=5)
parser.add_argument("--img_loss", type=float, default=0.1)
parser.add_argument("--joint_loss", type=float, default=1.0)
parser.add_argument("--pt_loss", type=float, default=0.1)
parser.add_argument("--force_loss", type=float, default=0.1)

parser.add_argument("--heatmap_size", type=float, default=0.1)
parser.add_argument("--temperature", type=float, default=1e-4)
parser.add_argument("--stdev", type=float, default=0.1)
parser.add_argument("--lr", type=float, default=1e-3)
parser.add_argument("--optimizer", type=str, default="adam")
parser.add_argument("--log_dir", default="log/k_dim_5")
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
detaset_folder = "/home/shikada/dataset/TUM/tomm_no_rotate/dual/all/"

def load_data(train='train', data_type='left'):
    # load dataset
    minmax = [args.vmin, args.vmax]
    images = np.load(detaset_folder + "{}/{}_images.npy".format(train, data_type))
    images = resize_img(images, (64, 64))
    images = images.transpose(0, 1, 4, 2, 3)
    images = normalization(images, (0, 255), minmax)
    poses = np.load(detaset_folder + "{}/{}_poses.npy".format(train, data_type))
    pose_bounds = np.load(detaset_folder +  "{}_pose_bounds.npy".format(data_type))
    poses = normalization(poses, pose_bounds, minmax)

    # skin sensor
    proximities = np.load(detaset_folder + "{}/{}_proximities.npy".format(train, data_type))
    proximitiy_bounds = np.load(detaset_folder + "{}_proximity_bounds.npy".format(data_type))
    proximities = normalization(proximities, proximitiy_bounds, minmax)
    return images, poses, proximities

# train data
left_image, left_poses, left_proximity = load_data(train='train', data_type='left')
right_image, right_poses, right_proximity = load_data(train='train', data_type='right')
train_dataset = MultimodalDataset(left_image, right_image, left_poses, right_poses, left_proximity, right_proximity, stdev=stdev)
train_loader = MultiEpochsDataLoader(
    train_dataset,
    batch_size=args.batch_size,
    shuffle=True,
    num_workers=args.n_worker,
    drop_last=False,
    pin_memory=True,
)
print("---train---")
print("left_image: ", left_image.shape, left_image.min(), left_image.max())
print("left_poses: ", left_poses.shape, left_poses.min(), left_poses.max())
print("left_proximity: ", left_proximity.shape, left_proximity.min(), left_proximity.max())
print("right_image: ", right_image.shape, right_image.min(), right_image.max())
print("right_poses: ", right_poses.shape, right_poses.min(), right_poses.max())
print("right_proximity: ", right_proximity.shape, right_proximity.min(), right_proximity.max())


# test data
left_image, left_poses, left_proximity = load_data(train='test', data_type='left')
right_image, right_poses, right_proximity = load_data(train='test', data_type='right')
test_dataset = MultimodalDataset(left_image, right_image, left_poses, right_poses, left_proximity, right_proximity, stdev=None)
test_loader = MultiEpochsDataLoader(
    test_dataset,
    batch_size=args.batch_size,
    shuffle=True,
    num_workers=args.n_worker,
    drop_last=False,
    pin_memory=True,
)
print("---test---")
print("left_image: ", left_image.shape, left_image.min(), left_image.max())
print("left_poses: ", left_poses.shape, left_poses.min(), left_poses.max())
print("left_proximity: ", left_proximity.shape, left_proximity.min(), left_proximity.max())
print("right_image: ", right_image.shape, right_image.min(), right_image.max())
print("right_poses: ", right_poses.shape, right_poses.min(), right_poses.max())
print("right_proximity: ", right_proximity.shape, right_proximity.min(), right_proximity.max())


# define model
model = SARNN(
    rec_dim=args.rec_dim,
    pose_dim=3,
    proximity_dim=1,
    k_dim=args.k_dim,
    heatmap_size=args.heatmap_size,
    temperature=args.temperature,
    im_size=[64, 64],
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

with tqdm(range(args.epoch)) as pbar_epoch:
    for epoch in pbar_epoch:
        # train and test
        train_loss = trainer.process_epoch(train_loader)
        with torch.no_grad():
            test_loss = trainer.process_epoch(test_loader, training=False)
        writer.add_scalar("Loss/train_loss", train_loss, epoch)
        writer.add_scalar("Loss/test_loss", test_loss, epoch)

        # early stop
        save_ckpt, _ = early_stop(test_loss)

        if save_ckpt:
            trainer.save(epoch, [train_loss, test_loss], save_name)

        # print process bar
        pbar_epoch.set_postfix(OrderedDict(train_loss=train_loss, test_loss=test_loss))
