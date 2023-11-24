#! /usr/bin/env python

import os
import sys
import glob
import numpy as np
import cv2
import torch
import rospy
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import PoseStamped
from skin_client.msg import SkinPatchData
from eipl.utils import normalization, deprocess_img, tensor2numpy, restore_args

import time

try:
    from libs.model import SARNN
except:
    sys.path.append("./libs/")
    from model import SARNN


class OnlineSARNN:
    def __init__(self, data_dir, model_name, inference_rate=10, max_episode_steps=270):
        self.model_name = model_name
        self.save_dir = os.path.join(data_dir, model_name)
        self.inference_rate = inference_rate
        self.max_episode_steps = max_episode_steps
        self.warmup_steps = 5

        self.left_orientation = [-0.499, 0.498, 0.482, 0.519]
        self.right_orientation = [0.490, 0.510, -0.493, 0.507]

        USE_CUDA = -1
        params = restore_args( os.path.join(self.save_dir, 'args.json') )
        self.params = params

        # load weight
        ckpt_file = sorted(glob.glob(os.path.join(self.save_dir, '*.pth')))
        latest = ckpt_file.pop()

        # load bounds
        data_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate/data/dual/all" + "/"
        self.minmax = [params["vmin"], params["vmax"]]
        self.left_pose_bounds = np.load(data_dir + "left_pose_bounds.npy")
        self.left_proximity_bounds = np.load(data_dir + "left_proximity_bounds.npy")
        self.right_pose_bounds = np.load(data_dir + "right_pose_bounds.npy")
        self.right_proximity_bounds = np.load(data_dir + "right_proximity_bounds.npy")

        # load sensor npy data
        dir_name = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate/npz/dual" + "/"
        bag_name = "tomm_tutorial_l_001.npz"
        npz = np.load(dir_name + bag_name)
        self.left_image = npz["left_images"]
        self.right_image = npz["right_images"]
        self.left_proximity = npz["left_proximities"]
        self.right_proximity = npz["right_proximities"]

        self.k_dim = params["k_dim"]
        self.prev_dec_pts = None

        # define model
        self.model = SARNN(
            rec_dim=params["rec_dim"],
            pose_dim=3,
            proximity_dim=10,
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
        self.left_target_pose_msg = PoseStamped()
        self.right_target_pose_msg = PoseStamped()
        self.reset()

        self.left_pose_sub = rospy.Subscriber("/tomm/teleop_left_hand/pose", PoseStamped, self.left_pose_callback)
        self.right_pose_sub = rospy.Subscriber("/tomm/teleop_right_hand/pose", PoseStamped, self.right_pose_callback)

        self.target_left_pos_pub = rospy.Publisher("/target/left_pose", PoseStamped, queue_size=1)
        self.target_right_pos_pub = rospy.Publisher("/target/right_pose", PoseStamped, queue_size=1)

    def reset(self):
        self.left_pose, self.right_pose = None, None
        self.steps = 0

#-----------------------------------------------------------------------------------------------------------------------------------------
    def run(self):
        self.reset()
        state = [None, None, None]
        rate = rospy.Rate(self.inference_rate)

        while not rospy.is_shutdown() and \
            (self.left_pose is None or self.right_pose is None or \
             self.left_proximity is None or self.right_proximity is None):
            rospy.logwarn_throttle(1.0, 'Waiting for data')
        
        rospy.logwarn('Real-time inference: Start')

        for i in range(len(self.left_image)):
            if rospy.is_shutdown():
                break

            t_start = time.time()
            y_left_image, y_right_image, y_left_pose, y_right_pose, y_left_proximity, y_right_proximity, \
                left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state = self.inference(state, i)
            rospy.logwarn_throttle(0.5, 'inference delay={0}s'.format(time.time() - t_start))

            # send to robot
            if self.steps > self.warmup_steps:
                self.publish(y_left_pose, self.left_orientation, self.left_target_pose_msg, self.target_left_pos_pub)
                self.publish(y_right_pose, self.right_orientation, self.right_target_pose_msg, self.target_right_pos_pub)
            # self.visualize(y_left_image, y_right_image, left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts)

            self.steps += 1  
            rate.sleep()

#-----------------------------------------------------------------------------------------------------------------------------------------
    def inference(self, state, i):
        # preprocess input data
        x_image = [self.process_image(self.left_image[i]), self.process_image(self.right_image[i])]
        x_pose = [self.process_signal(self.left_pose, self.left_pose_bounds),
                    self.process_signal(self.right_pose, self.right_pose_bounds)]
        x_proximity = [self.process_signal(self.left_proximity[i], self.left_proximity_bounds),
                        self.process_signal(self.right_proximity[i], self.right_proximity_bounds)]
        
        with torch.no_grad():
            y_image, y_pose, y_proximity, enc_pts, dec_pts, state = self.model(x_image, x_pose, x_proximity, state)
            
        # denormalization
        y_left_image = self.denorm_image(y_image[0][0])
        y_right_image = self.denorm_image(y_image[1][0])

        y_left_pose = self.denorm_signal(y_pose[0][0], self.left_pose_bounds)
        y_right_pose = self.denorm_signal(y_pose[1][0], self.right_pose_bounds)

        y_left_proximity = self.denorm_signal(y_proximity[0][0], self.left_proximity_bounds)
        y_right_proximity = self.denorm_signal(y_proximity[1][0], self.right_proximity_bounds)

        left_enc_pts = self.split_key_points(enc_pts[0][0])
        left_dec_pts = self.split_key_points(dec_pts[0][0])
        right_enc_pts = self.split_key_points(enc_pts[1][0])
        right_dec_pts = self.split_key_points(dec_pts[1][0])

        return y_left_image, y_right_image, y_left_pose, y_right_pose, y_left_proximity, y_right_proximity, \
                left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state


    def publish(self, y_pose, orientation, target_msg, target_pub):
        pose_list = y_pose.tolist()
        target_msg.pose.position.x = pose_list[0]
        target_msg.pose.position.y = pose_list[1]
        target_msg.pose.position.z = pose_list[2]
        target_msg.pose.orientation.x = orientation[0]
        target_msg.pose.orientation.y = orientation[1]
        target_msg.pose.orientation.z = orientation[2]
        target_msg.pose.orientation.w = orientation[3]
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = 'tomm/base_link'
        target_pub.publish(target_msg)


    def visualize(self, y_left_image, y_right_image, left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts):
        fac = 7
        x_left_image = cv2.resize(self.left_image, (fac*self.left_image.shape[1], fac*self.left_image.shape[0]))
        x_right_image = cv2.resize(self.right_image, (fac*self.right_image.shape[1], fac*self.right_image.shape[0]))
        y_left_image = cv2.resize(y_left_image, (fac*y_left_image.shape[1], fac*y_left_image.shape[0]))
        y_right_image = cv2.resize(y_right_image, (fac*y_right_image.shape[1], fac*y_right_image.shape[0]))
        left_enc_pts = left_enc_pts * fac
        left_dec_pts = left_dec_pts * fac
        right_enc_pts = right_enc_pts * fac
        right_dec_pts = right_dec_pts * fac

        for i in range(self.k_dim):
            cv2.circle(x_left_image, left_dec_pts[i].astype(np.int32), 2, (0,0,255), 5)      # RED
            cv2.circle(x_left_image, left_enc_pts[i].astype(np.int32), 2, (0,255,0), 5)      # GREEN
            cv2.circle(x_right_image, right_dec_pts[i].astype(np.int32), 2, (0,0,255), 5)    # RED
            cv2.circle(x_right_image, right_enc_pts[i].astype(np.int32), 2, (0,255,0), 5)    # GREEN

        x_image = np.concatenate((x_left_image, x_right_image), axis=1)
        y_image = np.concatenate((y_left_image, y_right_image), axis=1)

        result_image = np.concatenate((x_image, y_image), axis=1)
        cv2.imshow("Image", result_image)
        cv2.waitKey(3)

#-----------------------------------------------------------------------------------------------------------------------------------------
    def process_image(self, image):
        _image = cv2.resize(image, (64, 64))
        _image = _image.transpose(2, 0, 1)
        _image = normalization(_image, (0, 255), self.minmax)
        _image = np.expand_dims(_image, axis=0)
        return torch.from_numpy(_image).float()

    def process_signal(self, signal, bounds):
        _signal = normalization(signal, bounds, self.minmax)
        _signal = np.expand_dims(_signal, axis=0)
        return torch.from_numpy(_signal).float()
    
    def denorm_image(self, image):
        _image = tensor2numpy(image)
        _image = deprocess_img(_image, self.params["vmin"], self.params["vmax"])
        _image = _image.transpose(1, 2, 0)
        return _image
    
    def denorm_signal(self, signal, bounds):
        _signal = tensor2numpy(signal)
        _signal = normalization(_signal, self.minmax, bounds)
        return _signal
    
    def split_key_points(self, points, image_size=64):
        _points = tensor2numpy(points)
        _points = _points.reshape(-1, self.k_dim, 2) * image_size
        _points = np.clip(_points[0], 0, image_size)
        return _points
    
#-----------------------------------------------------------------------------------------------------------------------------------------
    def left_pose_callback(self, msg):
        self.left_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def right_pose_callback(self, msg):
        self.right_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

#-----------------------------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node("online_SARNN_image_node")
    rospy.loginfo("online_SARNN_image_node start")
    rospy.sleep(1.0)

    SARNN_motion = OnlineSARNN(
        data_dir="./log/k_dim_5", 
        model_name="20231121_0409_26", 
        inference_rate=10,
        max_episode_steps=20000)
    SARNN_motion.run()

    rospy.loginfo("online_SARNN_image_node shutdown")

if __name__ == "__main__":
    main()