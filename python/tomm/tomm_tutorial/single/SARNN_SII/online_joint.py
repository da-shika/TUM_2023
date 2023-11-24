#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, glob
import datetime
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2
import ros_numpy
from sensor_msgs.msg import Image, JointState

#import sys; sys.path.append("./")
from libs.model import SARNN
import torch
import torchvision.transforms as transforms
from eipl.utils import normalization, deprocess_img, tensor2numpy, restore_args, resize_img


class OnlineSARNN:
    def __init__(self, data_dir, model_name, inference_rate=10, max_episode_steps=180):

        self.loop_ct = 0
        self.input_param = 0.8
        self.model_name = model_name
        self.save_dir = os.path.join(data_dir, model_name)
        self.inference_rate = inference_rate
        self.max_episode_steps = max_episode_steps

        self.warmup_steps = 5

        cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)

        # Load the normalizations
        USE_CUDA = -1
        params = restore_args( os.path.join(self.save_dir, 'args.json') )
        self.params = params

        # load weight
        ckpt_file = sorted(glob.glob(os.path.join(self.save_dir, '*.pth')))
        latest = ckpt_file.pop()

        self.minmax = [params["vmin"], params["vmax"]]
        self.joint_bounds = np.load("ja_bounds.npy")
        
        # model of neural network
        self.model = SARNN(
            rec_dim=params["rec_dim"],
            joint_dim=15,
            k_dim=params["k_dim"],
            heatmap_size=params["heatmap_size"],
            temperature=params["temperature"],
            im_size=[64, 64]
        )

        if USE_CUDA == -1: 
            ckpt = torch.load(latest, map_location=torch.device('cpu') )        
        else:
            ckpt = torch.load(latest, map_location=torch.device('cuda') )

        self.model.load_state_dict(ckpt["model_state_dict"])
        self.model.eval()

        # setup output
        self.joint_pred_msg = JointState()
        self.joint_pred_msg.name = [
            'left_arm/joint_1',
            'left_arm/joint_2',
            'left_arm/joint_3',
            'left_arm/joint_4',
            'left_arm/joint_5',
            'left_arm/joint_6',
            'left_arm/joint_7',
            'right_arm/joint_1',
            'right_arm/joint_2',
            'right_arm/joint_3',
            'right_arm/joint_4',
            'right_arm/joint_5',
            'right_arm/joint_6',
            'right_arm/joint_7',
            "gripper_cmd"]

        self.idx_left = 10
        self.idx_right = 27
        self.reset()

        # setup the ros connections
        self.right_img_sub = rospy.Subscriber('/torobo/head/see3cam_right/camera/color/image_raw_local', Image, self.cameraRightCallback, queue_size=1)
        self.left_img_sub = rospy.Subscriber('/torobo/head/see3cam_left/camera/color/image_raw_local', Image, self.cameraLeftCallback, queue_size=1)
        self.joint_sub = rospy.Subscriber('/torobo/joint_states', JointState, self.jointStateCallback, queue_size=1)

        self.joint_pred_pub = rospy.Publisher('/joint_predictions', JointState, queue_size=1)

    def reset(self):
        # initial gripper_cmd is 1, state is 0
        # if send open  cmd(1), robot gripper state become 0 to 1.
        # if send close cmd(0), robot gripper state become 1 to 0.
        self.gripper_cmd, self.gripper_state = 1, 1
        self.left_img = None
        self.position = None
        self.external_effort = None
        self.gripper_state_pred = 0

        self.rollout_t_start = rospy.Time.now()
        self.steps = 0

        self.ja_real_norm = []
        self.ja_real_dep = []
        self.ja_pred_norm = []
        self.ja_pred_dep = []
        self.cmd_real_norm = []
        self.cmd_real_dep = []
        self.cmd_pred_norm = []
        self.cmd_pred_dep = []
        self.im_real = []
        self.im_rnn = []
        self.im_out = []
        self.rnn_state = []

#--------------------------------------------------------------------------------
    def execute(self):
        self.reset()
        state = None

        # wait for first data to arrive
        while (self.left_img is None or self.position is None) and not rospy.is_shutdown():
            rospy.logwarn_throttle(1.0, 'OnlineCAERNN.rollout(): Waiting for data')

        rospy.logwarn('OnlineCAERNN.execute(): Start')

        t_prev = rospy.Time.now()
        rate = rospy.Rate(self.inference_rate)

        while not rospy.is_shutdown() and self.steps < self.max_episode_steps:
            t_start = rospy.Time.now()

            # build the command, keep other arm static
            xv, xim, yv, yim, out_im, state = self.inference(state)

            # log time
            dt_infer = (rospy.Time.now() - t_start).to_sec()
            t_elapsed = (rospy.Time.now() - self.rollout_t_start).to_sec()
            t_loop = (t_start - t_prev).to_sec()
            t_prev = t_start

            # build pred ja
            ja_pred = yv[:14]
            cmd_pred = yv[14]

            # send to robot
            if self.steps > self.warmup_steps:
                self.publish(ja_pred, cmd_pred) 
            
            # visualize
            cv2.imshow("real_cae_rnn_img", out_im)
            cv2.waitKey(3)

            # saver
            #self.rnn_state.append(state[0].detach().cpu().numpy())
            self.rnn_state.append(np.stack([s.detach().cpu().numpy() for s in state]))

            self.steps += 1

            # real_joint_left
            # norm_joint_left            

            rate.sleep()

        ja_real_norm = np.array(self.ja_real_norm)
        ja_real_dep = np.array(self.ja_real_dep)
        ja_pred_norm = np.array(self.ja_pred_norm)
        ja_pred_dep = np.array(self.ja_pred_dep)
        im_real = np.array(self.im_real)
        im_rnn = np.array(self.im_rnn)
        im_out = np.array(self.im_out)
        rnn_state = np.array(self.rnn_state)

        np.savez("./output/npz/open_{}_{}_{}".format(self.model_name, datetime.date.today(), datetime.datetime.now().time().replace(microsecond=0)), \
                    ja_real_norm=ja_real_norm, ja_real_dep=ja_real_dep, ja_pred_norm=ja_pred_norm, ja_pred_dep=ja_pred_dep, \
                    im_real=im_real, im_rnn=im_rnn, rnn_state=rnn_state)
        
        return self.position

#--------------------------------------------------------------------------------
    def inference(self, state):
        # preprocess input data
        xv = self.processSignal(self.position, self.joint_bounds, self.minmax)
        xim = self.processLeftImage(self.left_img)
        xv = torch.squeeze(xv)

        with torch.no_grad():
            # saver real_dep
            self.ja_real_dep.append(self.position)

            if self.loop_ct > 0:
                _xv = self.input_param * xv[None,:] + (1.0-self.input_param)*self.prev_yv
            else:
                _xv = xv[None,:]

            yim, yv, ect_pts, dec_pts, state = self.model.forward(xim[None,:,:,:], _xv, state)
            self.loop_ct += 1
            self.prev_yim  = yim
            self.prev_yv = yv

            #saver pred_norm
            self.ja_pred_norm.append(tensor2numpy(yv))

        # saver real_norm
        self.ja_real_norm.append(tensor2numpy(xv))

        xim = self.deprocessImage(xim[None,:,:,:])
        yim = self.deprocessImage(yim[None,:,:,:])
        out_im = np.concatenate( (xim, yim), axis=1 )

        # saver im
        self.im_real.append(xim)
        self.im_rnn.append(yim)
        self.im_out.append(out_im)
        out_im = cv2.resize(out_im, dsize=(1200, 600))

        # deprocess data
        xv = self.deprocessSignal(xv[None,:], self.joint_bounds, self.minmax)
        yv = self.deprocessSignal(yv[None,:], self.joint_bounds, self.minmax)
        self.gripper_state_pred = yv[-1]

        #saver pred_dep
        self.ja_pred_dep.append(yv)

        return xv, xim, yv, yim, out_im, state


    def publish(self, lr_pred, cmd_preds):
        # emulate binary gripper feedback
        """
        if cmd_preds> 0.5:
            self.gripper_cmd = 1
        else:
            self.gripper_cmd = 0
        """

        #"""
        if self.gripper_state == 0 and cmd_preds> 0.9: # open
            self.gripper_cmd = 1
        if self.gripper_state == 0 and cmd_preds > 0.95: # fully open
            self.gripper_state = 1
        if self.gripper_state == 1 and cmd_preds < 0.4: #close
            self.gripper_cmd = 0
        if self.gripper_state == 1 and cmd_preds < 0.1: # fully close
            self.gripper_state = 0
        #"""
        

        pred_value = np.concatenate([lr_pred, np.array(self.gripper_cmd).reshape(1)])
        self.joint_pred_msg.position = pred_value.tolist()
        self.joint_pred_msg.header.stamp = rospy.Time.now()
        self.joint_pred_pub.publish(self.joint_pred_msg)

#--------------------------------------------------------------------------------
    def processSignal(self, sig, mean, minmax):
        sig = np.array([normalization(sig[i], (self.joint_bounds[0,i], self.joint_bounds[1,i]), (self.params['vmin'], self.params['vmax']) ) for i in range(sig.shape[-1])])
        return torch.from_numpy(sig).float()

    def processLeftImage(self, limg):
        shrink_img = cv2.resize(limg, dsize=(200, 150))
        xl, yl = 34, 10
        middle_crop_img = shrink_img[yl:yl+128, xl:xl+128]
        middle_crop_img = middle_crop_img.astype(np.float32) / 255
        middle_crop_img = cv2.resize(middle_crop_img, (64,64) )
        middle_crop_img = torch.from_numpy(middle_crop_img).permute(2,0,1)
        return middle_crop_img

    def deprocessSignal(self, sig, mean, minmax):
        sig = torch.squeeze(sig)
        sig = sig.detach().cpu().numpy()
        sig = np.array( [ (normalization(sig[i], (self.params['vmin'], self.params['vmax']), (self.joint_bounds[0,i], self.joint_bounds[1,i]) )) for i in range(sig.shape[-1])])

        if sig[6] < -10*np.pi/180:
            sig[6] = -10*np.pi/180
        if sig[-2] < -10*np.pi/180:
            sig[-2] = -10*np.pi/180

        return sig
    
    def deprocessImage(self, img):
        img = torch.squeeze(img)
        img = img.detach().cpu().numpy().transpose(1,2,0)
        img = deprocess_img( img, 0.0, 1.0 )
        return img

#--------------------------------------------------------------------------------
    def cameraLeftCallback(self, msg):
        img = ros_numpy.numpify(msg)
        self.left_img = img
    
    def cameraRightCallback(self, msg):
        img = ros_numpy.numpify(msg)
        self.right_img = img

    def jointStateCallback(self, msg):
        q, tau = np.array(msg.position), np.array(msg.effort)
        
        # left and right arm state
        q_left = q[self.idx_left:self.idx_left+7]
        q_right = q[self.idx_right:self.idx_right+7]
    
        # position and effort, use simulated gripper state
        #self.position = np.concatenate([q_left, q_right, [self.gripper_state]])
        self.position = np.concatenate([q_left, q_right, [self.gripper_state_pred]])
        

#--------------------------------------------------------------------------------
    def goto_pos(self, current_pos, goal_pos, time=10):
        loop_ct = 0
        nloop = time * self.inference_rate
        diff_pos = (goal_pos - current_pos) / nloop

        rate = rospy.Rate(self.inference_rate)

        target_pos = current_pos
        while not rospy.is_shutdown() and loop_ct < nloop:
            target_pos = target_pos + diff_pos
            import ipdb; ipdb.set_trace()

            self.publish(target_pos, cmd_pred=1.0)
            loop_ct += 1
            rate.sleep()



#--------------------------------------------------------------------------------
def main():
    rospy.init_node('online_SARNN_node')
    rospy.loginfo("online_SARNN_node start")
    rospy.sleep(1.0)

    data_dir = "./log"
    model_name = "20230710_1618_57"

    # setting files
    freq = 10
    initial_pos = np.deg2rad(np.array([-5.359, -36.949, -27.635, 106.7, 0.042, -27.334, 1.486,
                                       -2.172, -25.678, -34.265, 98.045, -5.838, -41.613, -6.362])) #SII
    

    # load model
    model = OnlineSARNN(data_dir=data_dir, model_name=model_name, 
                        inference_rate=freq, max_episode_steps=200)

    # inference loop
    for _ in range(10):
        user_input = input("Continue? - [default:yes] \n")
        if len(user_input) == 0:
            user_input = "yes"

        if user_input == "yes":
            current_pos = model.execute()
            model.goto_pos(current_pos[:-1], initial_pos, time=2)
        elif user_input == "no":
            print("terminated.")
            exit()
        else:
            print("Please input yes or no.")

    rospy.loginfo("online_SARNN_node shutdown")

if __name__=="__main__":
    main()
    