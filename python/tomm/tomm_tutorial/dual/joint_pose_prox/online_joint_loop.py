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
from eipl.utils import normalization, deprocess_img, tensor2numpy, restore_args


class OnlineSARNN:
    def __init__(self, data_dir, model_name, inference_rate=10, max_episode_steps=180):

        self.loop_ct = 0
        self.input_param = 0.8
        self.model_name = model_name
        self.save_dir = os.path.join(data_dir, model_name)
        self.inference_rate = inference_rate
        self.max_episode_steps = max_episode_steps

        self.warmup_steps = 5

        #cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)

        # Load the normalizations
        USE_CUDA = -1
        params = restore_args( os.path.join(self.save_dir, 'args.json') )
        self.params = params

        # load weight
        ckpt_file = sorted(glob.glob(os.path.join(self.save_dir, '*.pth')))
        latest = ckpt_file.pop()

        self.minmax = [params["vmin"], params["vmax"]]
        self.left_joint_bounds = np.load("../data/left_joint_bounds.npy")
        self.right_joint_bounds = np.load("../data/right_joint_bounds.npy")
        self.k_dim = params["k_dim"]
        self.prev_dec_pts = None
        
        # model of neural network
        self.model = SARNN(
            rec_dim=params["rec_dim"],
            joint_dim=8,
            k_dim=params["k_dim"],
            heatmap_size=params["heatmap_size"],
            temperature=params["temperature"],
            im_size=[64, 64],
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
            'left_arm/joint_1','left_arm/joint_2','left_arm/joint_3','left_arm/joint_4','left_arm/joint_5','left_arm/joint_6','left_arm/joint_7',
            'right_arm/joint_1','right_arm/joint_2','right_arm/joint_3','right_arm/joint_4','right_arm/joint_5','right_arm/joint_6','right_arm/joint_7',
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
        self.left_pos = None
        self.right_pos = None
        self.external_effort = None
        self.gripper_state_pred = 0

        self.rollout_t_start = rospy.Time.now()
        self.steps = 0

#--------------------------------------------------------------------------------
    def execute(self):
        self.reset()
        state = [None, None, None]

        # wait for first data to arrive
        while (self.left_img is None or self.position is None) and not rospy.is_shutdown():
            rospy.logwarn_throttle(1.0, 'OnlineSARNN.rollout(): Waiting for data')

        rospy.logwarn('OnlineSARNN.execute(): Start')

        t_prev = rospy.Time.now()
        rate = rospy.Rate(self.inference_rate)

        while not rospy.is_shutdown() and self.steps < self.max_episode_steps:
            # build the command, keep other arm static
            xv, xim, ja_pred, cmd_pred, yim, out_im, enc_pts, dec_pts, state = self.inference(state)

            # send to robot
            if self.steps > self.warmup_steps:
                self.publish(ja_pred, cmd_pred)

            # visualize
            self.visualize(64, enc_pts, dec_pts, xim, yim)

            self.steps += 1       
            rate.sleep()

        return self.position

#--------------------------------------------------------------------------------
    def inference(self, state):
        # preprocess input data
        xlv = self.processSignal(self.left_pos, self.left_joint_bounds, self.minmax)
        xrv = self.processSignal(self.right_pos, self.right_joint_bounds, self.minmax)
        xv = torch.stack([xlv,xrv],0)
        xlim, xrim = self.processLeftImage(self.left_img)
        xim = torch.stack([xlim,xrim],0)

        with torch.no_grad():
            if self.loop_ct > 0:
                _xv = self.input_param * xv[:,None,:] + (1.0-self.input_param) * self.prev_yv
            else:
                _xv = xv[:,None,:]

            yim, yv, ect_pts, dec_pts, state = self.model.forward(xim[:,None,:,:,:], _xv, state)
            self.loop_ct += 1
            self.prev_yim  = yim
            self.prev_yv = torch.stack([yv[0],yv[1]],0)
            self.prev_dec_pts = dec_pts

        xlim = self.deprocessImage(xlim[None,:,:,:])
        xrim = self.deprocessImage(xrim[None,:,:,:])
        xim = np.concatenate( (xlim, xrim), axis=1 )
        ylim = self.deprocessImage(yim[0])
        yrim = self.deprocessImage(yim[1])
        yim = np.concatenate( (ylim, yrim), axis=1 )
        out_im = np.concatenate( (xim, yim), axis=0 )

        # deprocess data
        xlv = self.deprocessSignal(xlv, self.left_joint_bounds, self.minmax)
        xrv = self.deprocessSignal(xrv, self.right_joint_bounds, self.minmax)

        _ylv, _yrv = yv[0], yv[1]
        ylv = self.deprocessSignal(_ylv, self.left_joint_bounds, self.minmax)
        yrv = self.deprocessSignal(_yrv, self.right_joint_bounds, self.minmax)
        yv = np.concatenate((ylv[:-1], yrv[:-1]), axis=0)
        self.gripper_state_pred = (ylv[-1] + yrv[-1]) / 2
        yc = (ylv[-1] + yrv[-1]) / 2

        return xv, xim, yv, yc, yim, out_im, ect_pts, dec_pts, state


    def publish(self, lr_pred, cmd_preds):
        # emulate binary gripper feedback
        """
        if cmd_preds> 0.5:
            self.gripper_cmd = 1
        else:
            self.gripper_cmd = 0
        """

        if self.gripper_state == 0 and cmd_preds> 0.9: # open
            self.gripper_cmd = 1
        if self.gripper_state == 0 and cmd_preds > 0.95: # fully open
            self.gripper_state = 1
        if self.gripper_state == 1 and cmd_preds < 0.6: #close
            self.gripper_cmd = 0
        if self.gripper_state == 1 and cmd_preds < 0.5: # fully close
            self.gripper_state = 0
        
        pred_value = np.concatenate([lr_pred, np.array(self.gripper_cmd).reshape(1)])
        self.joint_pred_msg.position = pred_value.tolist()
        self.joint_pred_msg.header.stamp = rospy.Time.now()
        self.joint_pred_pub.publish(self.joint_pred_msg)

    
    def visualize(self, img_size, enc_pts, dec_pts, xim, yim):
        xlim = xim[:,:64]
        xrim = xim[:,64:]
        ylim = yim[:,:64]
        yrim = yim[:,64:]

        left_enc_pts = tensor2numpy(enc_pts[0][0])
        left_dec_pts = tensor2numpy(dec_pts[0][0])
        left_enc_pts = left_enc_pts.reshape(-1, self.k_dim, 2) * img_size
        left_dec_pts = left_dec_pts.reshape(-1, self.k_dim, 2) * img_size
        left_enc_pts = np.clip(left_enc_pts[0], 0, img_size)
        left_dec_pts = np.clip(left_dec_pts[0], 0, img_size)

        right_enc_pts = tensor2numpy(enc_pts[1][0])
        right_dec_pts = tensor2numpy(dec_pts[1][0])
        right_enc_pts = right_enc_pts.reshape(-1, self.k_dim, 2) * img_size
        right_dec_pts = right_dec_pts.reshape(-1, self.k_dim, 2) * img_size
        right_enc_pts = np.clip(right_enc_pts[0], 0, img_size)
        right_dec_pts = np.clip(right_dec_pts[0], 0, img_size)

        # green: decoder (predictions)
        fac = 4
        xlim = cv2.resize(xlim, (fac*xlim.shape[1], fac*xlim.shape[0]))
        xrim = cv2.resize(xrim, (fac*xrim.shape[1], fac*xrim.shape[0]))
        ylim = cv2.resize(ylim, (fac*ylim.shape[1], fac*ylim.shape[0]))
        yrim = cv2.resize(yrim, (fac*yrim.shape[1], fac*yrim.shape[0]))
        #enc_pts[:,1] = img_size - enc_pts[:,1]
        #dec_pts[:,1] = img_size - dec_pts[:,1]
        left_enc_pts = left_enc_pts * fac
        left_dec_pts = left_dec_pts * fac
        right_enc_pts = right_enc_pts * fac
        right_dec_pts = right_dec_pts * fac
        for k in range(self.k_dim):
            cv2.circle(xlim, left_dec_pts[k].astype(np.int32), 2, (0,0,255), 5) # RED
            cv2.circle(xlim, left_enc_pts[k].astype(np.int32), 2, (0,255,0), 5) # GREEN
            cv2.circle(xrim, right_dec_pts[k].astype(np.int32), 2, (0,0,255), 5) # RED
            cv2.circle(xrim, right_enc_pts[k].astype(np.int32), 2, (0,255,0), 5) # GREEN

        xim = np.concatenate((xlim, xrim), axis=1)
        xim = cv2.resize(xim, dsize=(700, 350))
        cv2.imshow('xim', xim)
        yim = np.concatenate((ylim, yrim), axis=1)
        yim = cv2.resize(yim, dsize=(700, 350))
        cv2.imshow('yim', yim)
        cv2.waitKey(3)
#--------------------------------------------------------------------------------
    def processSignal(self, sig, joint_bounds, minmax):
        sig = np.array([normalization(sig[i], (joint_bounds[0,i], joint_bounds[1,i]), (minmax[0], minmax[1]) ) for i in range(sig.shape[-1])])
        return torch.from_numpy(sig).float()


    def processLeftImage(self, img):
        np_img = cv2.resize(img, dsize=(312, 264))
        xl, xr, y = 35, 163, 68
        left_crop_img = np_img[y:y+128, xl:xl+128].astype(np.float32) / 255
        left_crop_img = cv2.resize(left_crop_img, (64,64) )
        left_crop_img = torch.from_numpy(left_crop_img).permute(2,0,1)

        right_crop_img = np_img[y:y+128, xr:xr+128].astype(np.float32) / 255
        right_crop_img = cv2.resize(right_crop_img, (64,64) )
        right_crop_img = torch.from_numpy(right_crop_img).permute(2,0,1)
        return left_crop_img, right_crop_img


    def deprocessSignal(self, sig, joint_bounds, minmax):
        sig = torch.squeeze(sig)
        sig = sig.detach().cpu().numpy()
        sig = np.array( [ (normalization(sig[i], (minmax[0], minmax[1]), (joint_bounds[0,i], joint_bounds[1,i]) )) for i in range(sig.shape[-1])])
        # Hardware threshold
        if sig[6] < -10*np.pi/180:
            sig[6] = -10*np.pi/180
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
        self.left_pos = np.concatenate([q_left, [self.gripper_state_pred]])
        self.right_pos = np.concatenate([q_right, [self.gripper_state_pred]])
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

            self.publish(target_pos, cmd_preds=1.0)
            loop_ct += 1
            rate.sleep()



#--------------------------------------------------------------------------------
def main():
    rospy.init_node('online_SARNN_node')
    rospy.loginfo("online_SARNN_node start")
    rospy.sleep(1.0)

    data_dir = "./log/dual_im"
    model_name = "20230803_1448_17"

    # setting files
    freq = 10
    initial_pos = np.deg2rad(np.array([3.681, -30.247, -34.403, 98.547, -16.552, -43.447, 25.195,
                                       3.061, -25.93, -32.401, 101.3, -10.116, -44.151, 13.419]))       #IROS_pre
    

    # load model
    model = OnlineSARNN(data_dir=data_dir, model_name=model_name, 
                        inference_rate=freq, max_episode_steps=170)
    
    model.execute()

    """# go to initial position
    time.sleep(1)
    current_pos = model.position
    model.goto_pos(current_pos[:-1], initial_pos, time=2)

    # inference loop
    for _ in range(10):
        current_pos = model.execute()
        model.goto_pos(current_pos[:-1], initial_pos, time=2)
        time.sleep(4)"""

    """
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
    """

    rospy.loginfo("online_SARNN_node shutdown")

if __name__=="__main__":
    main()
    