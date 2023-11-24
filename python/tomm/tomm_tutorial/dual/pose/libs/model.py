#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import torch
import torch.nn as nn
from eipl.layer import SpatialSoftmax, InverseSpatialSoftmax
from eipl.utils import get_activation_fn


class HLSTMLayer(nn.Module):
    def __init__(self, in_dim, rec_dim, union_dim):
        super(HLSTMLayer, self).__init__()
        self.rec_dim = rec_dim
        self.left_rnn = nn.LSTMCell(in_dim, rec_dim)
        self.right_rnn = nn.LSTMCell(in_dim, rec_dim)
        self.union_rnn = nn.LSTMCell(rec_dim * 2, union_dim)
        self.union_out = nn.Linear(union_dim, rec_dim * 2)

    def forward(self, xl, xr, state=[None, None, None]):
        left_hid = self.left_rnn(xl, state[0])
        right_hid = self.right_rnn(xr, state[1])

        union_input = torch.concat((left_hid[1], right_hid[1]), axis=-1)    #変更
        union_hid = self.union_rnn(union_input, state[2])
        union_out = self.union_out(union_hid[0])
        _union_left, _union_right = torch.split(union_out, self.rec_dim, dim=-1)

        union_left = [left_hid[0], _union_left]     #変更
        union_right = [right_hid[0], _union_right]  #変更

        return [left_hid, right_hid], [union_left, union_right, union_hid]


class SELayer(nn.Module):
    def __init__(self, channel, reduction=16):
        super(SELayer, self).__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Sequential(
            nn.Linear(channel, channel // reduction, bias=False),
            nn.ReLU(inplace=True),
            nn.Linear(channel // reduction, channel, bias=False),
            nn.Sigmoid(),
        )

    def forward(self, x):
        b, c, _, _ = x.size()
        y = self.avg_pool(x).view(b, c)
        y = self.fc(y).view(b, c, 1, 1)
        return x * y.expand_as(x)


class SARNN(nn.Module):
    #:: SARNN
    """SARNN: Spatial Attention with Recurrent Neural Network.
    This model "explicitly" extracts positions from the image that are important to the task, such as the work object or arm position,
    and learns the time-series relationship between these positions and the robot's joint angles.
    The robot is able to generate robust motions in response to changes in object position and lighting.

    Arguments:
        rec_dim (int): The dimension of the recurrent state in the LSTM cell.
        k_dim (int, optional): The dimension of the attention points.
        joint_dim (int, optional): The dimension of the joint angles.
        temperature (float, optional): The temperature parameter for the softmax function.
        heatmap_size (float, optional): The size of the heatmap in the InverseSpatialSoftmax layer.
        kernel_size (int, optional): The size of the convolutional kernel.
        activation (str, optional): The name of activation function.
        im_size (list, optional): The size of the input image [height, width].
    """

    def __init__(
        self,
        rec_dim,
        k_dim=5,
        pose_dim=7,
        temperature=1e-4,
        heatmap_size=0.1,
        kernel_size=3,
        activation="lrelu",
        im_size=[128, 128],
    ):
        super(SARNN, self).__init__()

        self.k_dim = k_dim
        self.pose_dim = pose_dim

        if isinstance(activation, str):
            activation = get_activation_fn(activation, inplace=True)

        sub_im_size = [im_size[0] - 3 * (kernel_size - 1), im_size[1] - 3 * (kernel_size - 1)]
        self.temperature = temperature
        self.heatmap_size = heatmap_size

        # Positional Encoder
        self.pos_encoder = nn.Sequential(
            nn.Conv2d(3, 16, 3, 1, 0),  # Convolutional layer 1
            activation,
            nn.Conv2d(16, 32, 3, 1, 0),  # Convolutional layer 2
            activation,
            nn.Conv2d(32, self.k_dim, 3, 1, 0),  # Convolutional layer 3
            activation,
            SpatialSoftmax(
                width=sub_im_size[0],
                height=sub_im_size[1],
                temperature=self.temperature,
                normalized=True,
            ),  # Spatial Softmax layer
        )

        # Image Encoder
        self.im_encoder = nn.Sequential(
            nn.Conv2d(3, 16, 3, 1, 0),  # Convolutional layer 1
            activation,
            nn.Conv2d(16, 32, 3, 1, 0),  # Convolutional layer 2
            activation,
            nn.Conv2d(32, self.k_dim, 3, 1, 0),  # Convolutional layer 3
            activation,
        )
        
        vec_dim = pose_dim
        rec_in = vec_dim + self.k_dim * 2
        self.rec = HLSTMLayer(in_dim=rec_in, rec_dim=50, union_dim=10)

        # Joint and Point Decoder
        self.decoder_left_vec = nn.Sequential(
            nn.Linear(rec_dim, vec_dim), activation
        )
        
        self.decoder_left_point = nn.Sequential(
            nn.Linear(rec_dim, self.k_dim * 2), activation
        )
        
        self.decoder_right_vec = nn.Sequential(
            nn.Linear(rec_dim, vec_dim), activation
        )
        
        self.decoder_right_point = nn.Sequential(
            nn.Linear(rec_dim, self.k_dim * 2), activation
        )

        # Inverse Spatial Softmax
        self.issm = InverseSpatialSoftmax(
            width=sub_im_size[0],
            height=sub_im_size[1],
            heatmap_size=self.heatmap_size,
            normalized=True,
        )

        # Image Decoder
        self.decoder_image = nn.Sequential(
            nn.ConvTranspose2d(self.k_dim, 32, 3, 1, 0),  # Transposed Convolutional layer 1
            activation,
            nn.ConvTranspose2d(32, 16, 3, 1, 0),  # Transposed Convolutional layer 2
            activation,
            nn.ConvTranspose2d(16, 3, 3, 1, 0),  # Transposed Convolutional layer 3
            activation,
        )


    def encoder(self, xi):
        # Encode input image
        im_hid = self.im_encoder(xi)
        enc_pts, _ = self.pos_encoder(xi)
        enc_pts = enc_pts.reshape(-1, self.k_dim * 2)
        return im_hid, enc_pts

    def decoder(self, dec_pts, im_hid):
        # Reshape decoded points
        dec_pts_in = dec_pts.reshape(-1, self.k_dim, 2)
        heatmap = self.issm(dec_pts_in)  # Inverse Spatial Softmax
        hid = torch.mul(heatmap, im_hid)  # Multiply heatmap with image feature `im_hid`
        y_image = self.decoder_image(hid)  # Decode image
        
        return y_image


    def forward(self, xi, xpos, state=[None,None,None]):
        """
        Forward pass of the SARNN module.
        Predicts the image, joint angle, and attention at the next time based on the image and joint angle at time t.
        Predict the image, joint angles, and attention points for the next state (t+1) based on
        the image and joint angles of the current state (t).
        By inputting the predicted joint angles as control commands for the robot,
        it is possible to generate sequential motion based on sensor information.

        Arguments:
            xi (torch.Tensor): Input image tensor of shape (batch_size, channels, height, width).
            xj (torch.Tensor): Input joint tensor of shape (batch_size, input_dim).
            xpos (torch.Tensor): Input pose tensor of shape (batch_size, input_dim).
            state (tuple, optional): Initial hidden state and cell state of the LSTM cell.

        Returns:
            y_image (torch.Tensor): Decoded image tensor of shape (batch_size, channels, height, width).
            y_joint (torch.Tensor): Decoded joint prediction tensor of shape (batch_size, joint_dim).
            enc_pts (torch.Tensor): Encoded points tensor of shape (batch_size, k_dim * 2).
            dec_pts (torch.Tensor): Decoded points tensor of shape (batch_size, k_dim * 2).
            rnn_hid (tuple): Tuple containing the hidden state and cell state of the LSTM cell.
        """

        # image encoder
        left_im_hid, left_enc_pts = self.encoder(xi[0])
        right_im_hid, right_enc_pts = self.encoder(xi[1])

        # concat
        left_hid = torch.cat([xpos[1], left_enc_pts], -1)
        right_hid = torch.cat([xpos[1], right_enc_pts], -1)

        # rnn
        [left_hid, right_hid], state = self.rec(left_hid, right_hid, state)  # LSTM forward pass

        # vec predict
        y_left_pose = self.decoder_left_vec(left_hid[0])  # Decode vec prediction
        y_right_pose = self.decoder_right_vec(right_hid[0])  # Decode vec prediction

        left_dec_pts = self.decoder_left_point(left_hid[0])  # Decode points
        right_dec_pts = self.decoder_left_point(right_hid[0])  # Decode points


        y_left_image = self.decoder(left_dec_pts, left_im_hid)
        y_right_image = self.decoder(right_dec_pts, right_im_hid)
        
        return [y_left_image, y_right_image], [y_left_pose, y_right_pose], \
                [left_enc_pts, right_enc_pts], [left_dec_pts, right_dec_pts], state


if __name__ == "__main__":
    from torchinfo import summary

    input_dim = 17
    batch_size = 50
    model = HLSTMLayer(17, 50, 10)
    summary(model, input_size=[(batch_size, input_dim), (batch_size, input_dim)])
