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
        else:
            self.model.train()

        total_loss = 0.0
        for n_batch, ((x_img, x_pose, x_prox), (y_img, y_pose, y_prox)) in enumerate(data):
            x_img = x_img.to(self.device)
            y_img = y_img.to(self.device)
            x_pose = x_pose.to(self.device)
            y_pose = y_pose.to(self.device)
            x_prox = x_prox.to(self.device)
            y_prox = y_prox.to(self.device)

            state = None
            yi_list, ypose_list, yprox_list = [], [], []
            dec_pts_list, enc_pts_list = [], []
            T = x_img.shape[1]
            for t in range(T - 1):
                _yi_hat, _ypose_hat, _yprox_hat, enc_ij, dec_ij, state = self.model(x_img[:, t], x_pose[:, t], x_prox[:, t], state)
                yi_list.append(_yi_hat)
                ypose_list.append(_ypose_hat)
                yprox_list.append(_yprox_hat)
                enc_pts_list.append(enc_ij)
                dec_pts_list.append(dec_ij)

            yi_hat = torch.permute(torch.stack(yi_list), (1, 0, 2, 3, 4))
            ypose_hat = torch.permute(torch.stack(ypose_list), (1, 0, 2))
            yprox_hat = torch.permute(torch.stack(yprox_list), (1, 0, 2))

            img_loss = nn.MSELoss()(yi_hat, y_img[:, 1:]) * self.loss_weights[0]
            pose_loss = nn.MSELoss()(ypose_hat, y_pose[:, 1:]) * self.loss_weights[1]
            # Gradually change the loss value using the LossScheluder class.
            pt_loss = nn.MSELoss()(torch.stack(dec_pts_list[:-1]), torch.stack(enc_pts_list[1:])) * self.scheduler(self.loss_weights[2])
            prox_loss = nn.MSELoss()(yprox_hat, y_prox[:, 1:]) * self.loss_weights[3]

            loss = img_loss + pose_loss + pt_loss + prox_loss
            total_loss += loss.item()

            if training:
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                self.optimizer.step()

        return total_loss / (n_batch + 1)
