#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import torch
import torch.nn as nn
import torch.nn.functional as F
from eipl.layer import SpatialSoftmax, InverseSpatialSoftmax
from eipl.utils import get_activation_fn


class MTRNNCell(nn.Module):
    #:: MTRNNCell
    """Multiple Timescale RNN.

    Implements a form of Recurrent Neural Network (RNN) that operates with multiple timescales.
    This is based on the idea of hierarchical organization in human cognitive functions.

    Arguments:
        input_dim (int): Number of input features.
        fast_dim (int): Number of fast context neurons.
        slow_dim (int): Number of slow context neurons.
        fast_tau (float): Time constant value of fast context.
        slow_tau (float): Time constant value of slow context.
        activation (string, optional): If you set `None`, no activation is applied (ie. "linear" activation: `a(x) = x`).
        use_bias (Boolean, optional): whether the layer uses a bias vector. The default is False.
        use_pb (Boolean, optional): whether the recurrent uses a pb vector. The default is False.
    """

    def __init__(self,
                 input_dim_enc, input_dim_pose, input_dim_prox, 
                 fast_dim, slow_dim, union_dim,
                 fast_tau, slow_tau, union_tau, 
                 activation="tanh", att_activation="sigmoid",
                 use_bias=False, use_pb=False, epsilon=1e-7
                 ):
        super(MTRNNCell, self).__init__()

        self.input_dim_enc = input_dim_enc
        self.input_dim_pose = input_dim_pose
        self.input_dim_prox = input_dim_prox
        self.fast_dim = fast_dim
        self.slow_dim = slow_dim
        self.union_dim = union_dim
        self.fast_tau = fast_tau
        self.slow_tau = slow_tau
        self.union_tau = union_tau
        self.use_bias = use_bias
        self.use_pb = use_pb

        # Legacy string support for activation function.
        if isinstance(activation, str):
            self.activation = get_activation_fn(activation)
        else:
            self.activation = activation

        # Modality attention param
        self.kernel_att = nn.Parameter(torch.Tensor(self.slow_dim, 2))
        self.bias_att = nn.Parameter(torch.Tensor(2))
        self.activation_att = getattr(F, att_activation)

        def _make_one_side_MTRNN():
            # Input Layers
            pose_i2f = nn.Linear(self.input_dim_pose, fast_dim, bias=use_bias)
            enc_i2f = nn.Linear(self.input_dim_enc, fast_dim, bias=use_bias)
            prox_i2f = nn.Linear(self.input_dim_prox, fast_dim, bias=use_bias)

            # Fast context layer
            pose_f2f = nn.Linear(fast_dim, fast_dim, bias=False)
            pose_f2s = nn.Linear(fast_dim, slow_dim, bias=use_bias)
            enc_f2f = nn.Linear(fast_dim, fast_dim, bias=False)
            enc_f2s = nn.Linear(fast_dim, slow_dim, bias=use_bias)
            prox_f2f = nn.Linear(fast_dim, fast_dim, bias=False)
            prox_f2s = nn.Linear(fast_dim, slow_dim, bias=use_bias)

            # Slow context layer
            s2s = nn.Linear(slow_dim, slow_dim, bias=False)
            pose_s2f = nn.Linear(slow_dim, fast_dim, bias=use_bias)
            enc_s2f = nn.Linear(slow_dim, fast_dim, bias=use_bias)
            prox_s2f = nn.Linear(slow_dim, fast_dim, bias=use_bias)
            s2u = nn.Linear(slow_dim, union_dim, bias=use_bias)

            # Union context layer
            u2s = nn.Linear(union_dim, slow_dim, bias=use_bias)

            return pose_i2f, enc_i2f, prox_i2f, pose_f2f, pose_f2s, enc_f2f, enc_f2s, prox_f2f, prox_f2s,\
                    s2s, pose_s2f, enc_s2f, prox_s2f, s2u, u2s

        self.left_pose_i2f, self.left_enc_i2f, self.left_prox_i2f, \
            self.left_pose_f2f, self.left_pose_f2s, self.left_enc_f2f, self.left_enc_f2s, self.left_prox_f2f, self.left_prox_f2s, \
            self.left_s2s, self.left_pose_s2f, self.left_enc_s2f, self.left_prox_s2f, self.left_s2u, self.left_u2s = _make_one_side_MTRNN()
        
        self.right_pose_i2f, self.right_enc_i2f, self.right_prox_i2f, \
            self.right_pose_f2f, self.right_pose_f2s, self.right_enc_f2f, self.right_enc_f2s, self.right_prox_f2f, self.right_prox_f2s, \
            self.right_s2s, self.right_pose_s2f, self.right_enc_s2f, self.right_prox_s2f, self.right_s2u, self.right_u2s = _make_one_side_MTRNN()
        
        # Union context layer
        self.u2u = nn.Linear(union_dim, union_dim, bias=False)


    def forward(self, xl_pose, xl_enc, xl_prox, xr_pose, xr_enc, xr_prox, state=None, pb=None):
        """Forward propagation of the MTRNN.

        Arguments:
            x (torch.Tensor): Input tensor of shape (batch_size, input_dim).
            state (list): Previous states (left_h_fast, left_h_slow, left_u_fast, left_u_slow, 
                                            right_h_fast, right_h_slow, right_u_fast, right_u_slow, h_union, u_union), 
                    each of shape (batch_size, context_dim).
                   If None, initialize states to zeros.
            pb (bool): pb vector. Used if self.use_pb is set to True.

        Returns:
            new_left_h_fast (torch.Tensor): Updated fast context state.
            new_left_h_slow (torch.Tensor): Updated slow context state.
            new_right_h_fast (torch.Tensor): Updated fast context state.
            new_right_h_slow (torch.Tensor): Updated slow context state.
            new_h_union (torch.Tensor): Updated union context state.
            new_left_u_fast (torch.Tensor): Updated fast internal state.
            new_left_u_slow (torch.Tensor): Updated slow internal state.
            new_right_u_fast (torch.Tensor): Updated fast internal state.
            new_right_u_slow (torch.Tensor): Updated slow internal state.
            new_u_union (torch.Tensor): Updated union internal state.
        """
        batch_size = xl_pose.shape[0]
        if state is not None:
            prev_left_pose_h_fast, prev_left_enc_h_fast, prev_left_prox_h_fast, prev_left_h_slow, \
                prev_left_pose_u_fast, prev_left_enc_u_fast, prev_left_prox_u_fast, prev_left_u_slow, \
                prev_right_pose_h_fast, prev_right_enc_h_fast, prev_right_prox_h_fast, prev_right_h_slow, \
                prev_right_pose_u_fast, prev_right_enc_u_fast, prev_right_prox_u_fast, prev_right_u_slow, \
                prev_h_union, prev_u_union = state
        else:
            device = xl_pose.device
            def _make_zero_prev():
                left_pose_fast = torch.zeros(batch_size, self.fast_dim).to(device)
                left_enc_fast = torch.zeros(batch_size, self.fast_dim).to(device)
                left_prox_fast = torch.zeros(batch_size, self.fast_dim).to(device)
                left_slow = torch.zeros(batch_size, self.slow_dim).to(device)
                right_pose_fast = torch.zeros(batch_size, self.fast_dim).to(device)
                right_enc_fast = torch.zeros(batch_size, self.fast_dim).to(device)
                right_prox_fast = torch.zeros(batch_size, self.fast_dim).to(device)
                right_slow = torch.zeros(batch_size, self.slow_dim).to(device)
                union = torch.zeros(batch_size, self.union_dim).to(device)
                return left_pose_fast, left_enc_fast, left_prox_fast, left_slow, right_pose_fast, right_enc_fast, right_prox_fast, right_slow, union
            
            # context state
            prev_left_pose_h_fast, prev_left_enc_h_fast, prev_left_prox_h_fast, prev_left_h_slow, \
                prev_right_pose_h_fast, prev_right_enc_h_fast, prev_right_prox_h_fast, prev_right_h_slow, prev_h_union = _make_zero_prev()
            
            # internal state
            prev_left_pose_u_fast, prev_left_enc_u_fast, prev_left_prox_u_fast, prev_left_u_slow, \
                prev_right_pose_u_fast, prev_right_enc_u_fast, prev_right_prox_u_fast, prev_right_u_slow, prev_u_union = _make_zero_prev()
            

        def _set_one_side_MTRNN(prev_pose_u_fast, pose_i2f, x_pose, pose_f2f, prev_pose_h_fast, pose_s2f, prev_h_slow,\
                                prev_enc_u_fast, enc_i2f, x_enc, enc_f2f, prev_enc_h_fast, enc_s2f,\
                                prev_prox_u_fast, prox_i2f, x_prox, prox_f2f, prev_prox_h_fast, prox_s2f,\
                                pose_f2s, enc_f2s, prox_f2s, s2s, u2s, prev_u_slow):
            
            new_pose_u_fast = (1.0 - 1.0/self.fast_tau) * prev_pose_u_fast + \
                                1.0/self.fast_tau * (pose_i2f(x_pose) + pose_f2f(prev_pose_h_fast) + pose_s2f(prev_h_slow))
            new_enc_u_fast = (1.0 - 1.0/self.fast_tau) * prev_enc_u_fast + \
                                1.0/self.fast_tau * (enc_i2f(x_enc) + enc_f2f(prev_enc_h_fast) + enc_s2f(prev_h_slow))
            new_prox_u_fast = (1.0 - 1.0/self.fast_tau) * prev_prox_u_fast + \
                                1.0/self.fast_tau * (prox_i2f(x_prox) + prox_f2f(prev_prox_h_fast) + prox_s2f(prev_h_slow))
            
            # Modality attention
            _att_h = torch.matmul(prev_h_slow, self.kernel_att)
            _att_h = _att_h + self.bias_att
            _att_h = self.activation_att(_att_h)

            _att_h1, _att_h3 = torch.split(_att_h, [1,1], axis=-1)
            att_h1 = _att_h1/(_att_h1+_att_h3+self.epsilon)
            att_h3 = _att_h3/(_att_h1+_att_h3+self.epsilon)

            _input_slow = pose_f2s(prev_pose_h_fast) + enc_f2s(prev_enc_h_fast*att_h1)+ prox_f2s(prev_prox_h_fast*att_h3) + \
                            s2s(prev_h_slow) + u2s(prev_h_union)
            if pb is not None:
                _input_slow += pb
            
            new_u_slow = (1.0 - 1.0/self.slow_tau) * prev_u_slow + 1.0 / self.slow_tau * _input_slow
            
            new_pose_h_fast = self.activation(new_pose_u_fast)
            new_enc_h_fast = self.activation(new_enc_u_fast)
            new_prox_h_fast = self.activation(new_prox_u_fast)
            new_h_slow = self.activation(new_u_slow)

            return new_pose_h_fast, new_enc_h_fast, new_prox_h_fast, new_h_slow, new_pose_u_fast, new_enc_u_fast, new_prox_u_fast, new_u_slow
        
        # left MTRNN
        new_left_pose_h_fast, new_left_enc_h_fast, new_left_prox_h_fast, new_left_h_slow, \
            new_left_pose_u_fast, new_left_enc_u_fast, new_left_prox_u_fast, new_left_u_slow =\
                _set_one_side_MTRNN(prev_left_pose_u_fast, self.left_pose_i2f, xl_pose, self.left_pose_f2f, prev_left_pose_h_fast, self.left_pose_s2f, prev_left_h_slow,\
                                    prev_left_enc_u_fast, self.left_enc_i2f, xl_enc, self.left_enc_f2f, prev_left_enc_h_fast, self.left_enc_s2f,\
                                    prev_left_prox_u_fast, self.left_prox_i2f, xl_prox, self.left_prox_f2f, prev_left_prox_h_fast, self.left_prox_s2f,\
                                    self.left_pose_f2s, self.left_enc_f2s, self.left_prox_f2s, self.left_s2s, self.left_u2s, prev_left_u_slow)

        # right MTRNN
        new_right_pose_h_fast, new_right_enc_h_fast, new_right_prox_h_fast, new_right_h_slow, \
            new_right_pose_u_fast, new_right_enc_u_fast, new_right_prox_u_fast, new_right_u_slow =\
                _set_one_side_MTRNN(prev_right_pose_u_fast, self.right_pose_i2f, xr_pose, self.right_pose_f2f, prev_right_pose_h_fast, self.right_pose_s2f, prev_right_h_slow,\
                                    prev_right_enc_u_fast, self.right_enc_i2f, xr_enc, self.right_enc_f2f, prev_right_enc_h_fast, self.right_enc_s2f,\
                                    prev_right_prox_u_fast, self.right_prox_i2f, xr_prox, self.right_prox_f2f, prev_right_prox_h_fast, self.right_prox_s2f,\
                                    self.right_pose_f2s, self.right_enc_f2s, self.right_prox_f2s, self.right_s2s, self.right_u2s, prev_right_u_slow)

        # union MTRNN
        new_u_union = (1.0 - 1.0/self.union_tau) * prev_u_union + \
                        1.0/self.slow_tau * (self.left_s2u(prev_left_h_slow) + self.right_s2u(prev_right_h_slow) + self.u2u(prev_h_union))
        new_h_union = self.activation(new_u_union)

        return [new_left_pose_h_fast, new_left_enc_h_fast, new_left_prox_h_fast, new_left_h_slow, \
                    new_left_pose_u_fast, new_left_enc_u_fast, new_left_prox_u_fast, new_left_u_slow], \
                [new_right_pose_h_fast, new_right_enc_h_fast, new_right_prox_h_fast, new_right_h_slow, \
                    new_right_pose_u_fast, new_right_enc_u_fast, new_right_prox_u_fast, new_right_u_slow], \
                [new_left_pose_h_fast, new_left_enc_h_fast, new_left_prox_h_fast, new_left_h_slow, \
                    new_left_pose_u_fast, new_left_enc_u_fast, new_left_prox_u_fast, new_left_u_slow, \
                    new_right_pose_h_fast, new_right_enc_h_fast, new_right_prox_h_fast, new_right_h_slow, \
                    new_right_pose_u_fast, new_right_enc_u_fast, new_right_prox_u_fast, new_right_u_slow, new_h_union, new_u_union]


#------------------------------------------------------------------------------------------------------------------
class MTRNN(nn.Module):
    def __init__(
        self,
        fast_dim=20, slow_dim=5, union_dim=4,
        fast_tau=3, slow_tau=6, union_tau=12, 
        k_dim=5,
        pose_dim=3,
        proximity_dim=1,
        temperature=1e-4, heatmap_size=0.1, kernel_size=3, activation="lrelu",
        im_size=[128, 128],
    ):
        super(MTRNN, self).__init__()

        self.fast_dim = fast_dim, 
        self.slow_dim = slow_dim, 
        self.union_dim = union_dim,
        self.fast_tau = fast_tau, 
        self.slow_tau = slow_tau, 
        self.union_tau = union_tau, 
        self.k_dim = k_dim
        self.pose_dim = pose_dim
        self.proximity_dim = proximity_dim

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

        # Reccurent
        self.rec = MTRNNCell(input_dim_enc_pt=self.k_dim*2, input_dim_pose=self.pose_dim, input_dim_prox=self.proximity_dim, 
                            fast_dim=fast_dim, slow_dim=slow_dim, union_dim=union_dim,
                            fast_tau=fast_tau, slow_tau=slow_tau, union_tau=union_tau, 
                            activation="tanh",
                            use_bias=False, use_pb=False,)
        
        self.decoder_left_pose = nn.Sequential(nn.Linear(fast_dim, pose_dim), activation)
        self.decoder_left_point = nn.Sequential(nn.Linear(fast_dim, self.k_dim*2), activation)
        self.decoder_left_prox = nn.Sequential(nn.Linear(fast_dim, proximity_dim), activation)

        self.decoder_right_pose = nn.Sequential(nn.Linear(fast_dim, pose_dim), activation)
        self.decoder_right_point = nn.Sequential(nn.Linear(fast_dim, self.k_dim*2), activation)
        self.decoder_right_prox = nn.Sequential(nn.Linear(fast_dim, proximity_dim), activation)

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
    
    
    def forward(self, xi, xpos, xprox, state=None):
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
            xpos (torch.Tensor): Input force tensor of shape (batch_size, input_dim).
            xf (torch.Tensor): Input force tensor of shape (batch_size, input_dim).
            xp (torch.Tensor): Input proximity tensor of shape (batch_size, input_dim).
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

        # rnn
        left_hid, right_hid, state = self.rec(xpos[0], left_enc_pts, xprox[0], xpos[1], right_enc_pts, xprox[1], state)

        # vec predict
        y_left_pose = self.decoder_left_pose(left_hid[0])
        left_dec_pts = self.decoder_left_point(left_hid[0])
        y_left_prox = self.decoder_left_prox(left_hid[0])

        y_right_pose = self.decoder_right_pose(right_hid[0])
        right_dec_pts = self.decoder_right_point(right_hid[0])
        y_right_prox = self.decoder_right_prox(right_hid[0])

        y_left_image = self.decoder(left_dec_pts, left_im_hid)
        y_right_image = self.decoder(right_dec_pts, right_im_hid)

        
        return [y_left_image, y_right_image], [y_left_pose, y_right_pose], [y_left_prox, y_right_prox], \
                [left_enc_pts, right_enc_pts], [left_dec_pts, right_dec_pts], state