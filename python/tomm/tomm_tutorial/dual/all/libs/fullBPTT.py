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
            (x_left_img, x_right_img, x_left_joint, x_right_joint, x_left_pose, x_right_pose, \
             x_left_force, x_right_force, x_left_proximity, x_right_proximity),
            (y_left_img, y_right_img, y_left_joint, y_right_joint, y_left_pose, y_right_pose, \
             y_left_force, y_right_force, y_left_proximity, y_right_proximity),
        ) in enumerate(data):
            x_left_img = x_left_img.to(self.device)
            x_right_img = x_right_img.to(self.device)
            x_left_joint = x_left_joint.to(self.device)
            x_right_joint = x_right_joint.to(self.device)
            x_left_pose = x_left_pose.to(self.device)
            x_right_pose = x_right_pose.to(self.device)

            x_left_force = x_left_force.to(self.device)
            x_right_force = x_right_force.to(self.device)
            x_left_proximity = x_left_proximity.to(self.device)
            x_right_proximity = x_right_proximity.to(self.device)

            y_left_img = y_left_img.to(self.device)
            y_right_img = y_right_img.to(self.device)
            y_left_joint = y_left_joint.to(self.device)
            y_right_joint = y_right_joint.to(self.device)
            y_left_pose = y_left_pose.to(self.device)
            y_right_pose = y_right_pose.to(self.device)

            y_left_force = y_left_force.to(self.device)
            y_right_force = y_right_force.to(self.device)
            y_left_proximity = y_left_proximity.to(self.device)
            y_right_proximity = y_right_proximity.to(self.device)

            state = [None, None, None]
            y_left_img_list, y_right_img_list = [], []
            y_left_joint_list, y_right_joint_list = [], []
            y_left_pose_list, y_right_pose_list = [], []
            y_left_force_list, y_right_force_list = [], []
            y_left_proximity_list, y_right_proximity_list = [], []
            left_enc_pts_list, right_enc_pts_list = [], []
            left_dec_pts_list, right_dec_pts_list = [], []

            T = x_left_img.shape[1]
            for t in range(T - 1):
                x_img = [x_left_img[:, t], x_right_img[:, t]]
                x_joint = [x_left_joint[:, t], x_right_joint[:, t]]
                x_pose = [x_left_pose[:, t], x_right_pose[:, t]]
                x_force = [x_left_force[:, t], x_right_force[:, t]]
                x_proximity = [x_left_proximity[:, t], x_right_proximity[:, t]]

                _yi_hat, _yj_hat, _ypose_hat, _yf_hat, _yp_hat, enc_ij, dec_ij, state = self.model(x_img, x_joint, x_pose, x_force, x_proximity, state)
                y_left_img_list.append(_yi_hat[0])
                y_right_img_list.append(_yi_hat[1])
                y_left_joint_list.append(_yj_hat[0])
                y_right_joint_list.append(_yj_hat[1])
                y_left_pose_list.append(_ypose_hat[0])
                y_right_pose_list.append(_ypose_hat[1])

                y_left_force_list.append(_yf_hat[0])
                y_right_force_list.append(_yf_hat[1])
                y_left_proximity_list.append(_yp_hat[0])
                y_right_proximity_list.append(_yp_hat[1])
                left_enc_pts_list.append(enc_ij[0])
                right_enc_pts_list.append(enc_ij[1])
                left_dec_pts_list.append(dec_ij[0])
                right_dec_pts_list.append(dec_ij[1])

            y_left_img_hat = torch.permute(torch.stack(y_left_img_list), (1, 0, 2, 3, 4))
            y_right_img_hat = torch.permute(torch.stack(y_right_img_list), (1, 0, 2, 3, 4))
            y_left_joint_hat = torch.permute(torch.stack(y_left_joint_list), (1, 0, 2))
            y_right_joint_hat = torch.permute(torch.stack(y_right_joint_list), (1, 0, 2))
            y_left_pose_hat = torch.permute(torch.stack(y_left_pose_list), (1, 0, 2))
            y_right_pose_hat = torch.permute(torch.stack(y_right_pose_list), (1, 0, 2))

            y_left_force_hat = torch.permute(torch.stack(y_left_force_list), (1, 0, 2))
            y_right_force_hat = torch.permute(torch.stack(y_right_force_list), (1, 0, 2))
            y_left_proximity_hat = torch.permute(torch.stack(y_left_proximity_list), (1, 0, 2))
            y_right_proximity_hat = torch.permute(torch.stack(y_right_proximity_list), (1, 0, 2))
            left_enc_pts = torch.stack(left_enc_pts_list[1:])
            right_enc_pts = torch.stack(right_enc_pts_list[1:])
            left_dec_pts = torch.stack(left_dec_pts_list[:-1])
            right_dec_pts = torch.stack(right_dec_pts_list[:-1])

            left_img_loss = nn.MSELoss()(y_left_img_hat, y_left_img[:, 1:]) * self.loss_weights[0]
            right_img_loss = nn.MSELoss()(y_right_img_hat, y_right_img[:, 1:]) * self.loss_weights[0]
            left_joint_loss =nn.MSELoss()(y_left_joint_hat, y_left_joint[:, 1:]) * self.loss_weights[1]
            right_joint_loss =nn.MSELoss()(y_right_joint_hat, y_right_joint[:, 1:]) * self.loss_weights[1]
            left_pose_loss =nn.MSELoss()(y_left_pose_hat, y_left_pose[:, 1:]) * self.loss_weights[1]
            right_pose_loss =nn.MSELoss()(y_right_pose_hat, y_right_pose[:, 1:]) * self.loss_weights[1]
        
            left_pt_loss = nn.MSELoss()(left_enc_pts, left_dec_pts) * self.scheduler(
                self.loss_weights[2]
            )
            right_pt_loss = nn.MSELoss()(right_enc_pts, right_dec_pts) * self.scheduler(
                self.loss_weights[2]
            )
            left_force_loss =nn.MSELoss()(y_left_force_hat, y_left_force[:, 1:]) * self.loss_weights[3]
            right_force_loss =nn.MSELoss()(y_right_force_hat, y_right_force[:, 1:]) * self.loss_weights[3]
            left_proximity_loss =nn.MSELoss()(y_left_proximity_hat, y_left_proximity[:, 1:]) * self.loss_weights[3]
            right_proximity_loss =nn.MSELoss()(y_right_proximity_hat, y_right_proximity[:, 1:]) * self.loss_weights[3]
            loss = (
                left_img_loss + right_img_loss + 
                left_joint_loss + right_joint_loss + 
                left_pose_loss + right_pose_loss + 
                left_pt_loss + right_pt_loss + 
                left_force_loss + right_force_loss + 
                left_proximity_loss + right_proximity_loss
            )
            total_loss += loss.item()

            if training:
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                self.optimizer.step()

        return total_loss / (n_batch + 1)
