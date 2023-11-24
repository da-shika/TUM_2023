#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import torch
import torch.nn as nn
from eipl.utils import LossScheduler


class fullBPTTtrainer:
    """
    Helper class to train recurrent neural networks with numpy sequences

    Args:
        traindata (np.array): list of np.array. First diemension should be time steps
        model (torch.nn.Module): rnn model
        optimizer (torch.optim): optimizer
        input_param (float): input parameter of sequential generation. 1.0 means open mode.
    """

    def __init__(self, model, optimizer, loss_weights=[1.0, 1.0], device="cpu"):
        self.device = device
        self.optimizer = optimizer
        self.loss_weights = loss_weights
        self.scheduler = LossScheduler(decay_end=1000, curve_name="s")
        self.model = model.to(self.device)

    def save(self, epoch, loss, savename):
        torch.save(
            {
                "epoch": epoch,
                "model_state_dict": self.model.state_dict(),
                #'optimizer_state_dict': self.optimizer.state_dict(),
                "train_loss": loss[0],
                "test_loss": loss[1],
            },
            savename,
        )

    def process_epoch(self, data, training=True):
        if not training:
            self.model.eval()

        total_loss = 0.0
        for n_batch, (
            (x_img, x_joint, x_pose, x_force, x_proximity),
            (y_img, y_joint, y_pose, y_force, y_proximity),
        ) in enumerate(data):
            x_img = x_img.to(self.device)
            x_joint = x_joint.to(self.device)
            x_pose = x_pose.to(self.device)
            x_force = x_force.to(self.device)
            x_proximity = x_proximity.to(self.device)

            y_img = y_img.to(self.device)
            y_joint = y_joint.to(self.device)
            y_pose = y_pose.to(self.device)
            y_force = y_force.to(self.device)
            y_proximity = y_proximity.to(self.device)

            state = None
            y_img_list = []
            y_joint_list = []
            y_pose_list = []
            y_force_list = []
            y_proximity_list = []
            enc_pts_list = []
            dec_pts_list = []

            T = x_img.shape[1]
            for t in range(T - 1):
                x_img = x_img[:, t]
                x_joint = x_joint[:, t]
                x_pose = x_pose[:, t]
                x_force = x_force[:, t]
                x_proximity = x_proximity[:, t]

                _yi_hat, _yj_hat, _ypose_hat, _yf_hat, _yp_hat, enc_ij, dec_ij, state = self.model(x_img, x_joint, x_pose, x_force, x_proximity, state)
                y_img_list.append(_yi_hat)
                y_joint_list.append(_yj_hat)
                y_pose_list.append(_ypose_hat)

                y_force_list.append(_yf_hat)
                y_proximity_list.append(_yp_hat)
                enc_pts_list.append(enc_ij)
                dec_pts_list.append(dec_ij)

            y_img_hat = torch.permute(torch.stack(y_img_list), (1, 0, 2, 3, 4))
            y_joint_hat = torch.permute(torch.stack(y_joint_list), (1, 0, 2))
            y_pose_hat = torch.permute(torch.stack(y_pose_list), (1, 0, 2))

            y_force_hat = torch.permute(torch.stack(y_force_list), (1, 0, 2))
            y_proximity_hat = torch.permute(torch.stack(y_proximity_list), (1, 0, 2))
            enc_pts = torch.stack(enc_pts_list[1:])
            dec_pts = torch.stack(dec_pts_list[:-1])

            img_loss = nn.MSELoss()(y_img_hat, y_img[:, 1:]) * self.loss_weights[0]
            joint_loss =nn.MSELoss()(y_joint_hat, y_joint[:, 1:]) * self.loss_weights[1]
            pose_loss =nn.MSELoss()(y_pose_hat, y_pose[:, 1:]) * self.loss_weights[1]
        
            pt_loss = nn.MSELoss()(enc_pts, dec_pts) * self.scheduler(self.loss_weights[2])
            force_loss =nn.MSELoss()(y_force_hat, y_force[:, 1:]) * self.loss_weights[3]
            proximity_loss =nn.MSELoss()(y_proximity_hat, y_proximity[:, 1:]) * self.loss_weights[3]
            loss = (
                img_loss + 
                joint_loss + pose_loss + 
                pt_loss + 
                force_loss + proximity_loss
            )
            total_loss += loss.item()

            if training:
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                self.optimizer.step()

        return total_loss / (n_batch + 1)
