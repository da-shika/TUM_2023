#! /usr/bin/env python

import os
import sys
import glob
import numpy as np
import cv2
import torch
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from skin_client.msg import SkinPatchData
from eipl.utils import normalization, deprocess_img, tensor2numpy, restore_args

try:
    from libs.model import SARNN
except:
    sys.path.append("./libs/")
    from model import SARNN


class OnlineSARNNImage:
    def __init__(self, data_dir, model_name, inference_rate=10):
        self.model_name = model_name
        self.save_dir = os.path.join(data_dir, model_name)
        self.inference_rate = inference_rate

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

        self.reset()

        self.left_pose_sub = rospy.Subscriber("/tomm/teleop_left_hand/pose", PoseStamped, self.left_pose_callback)
        self.right_pose_sub = rospy.Subscriber("/tomm/teleop_right_hand/pose", PoseStamped, self.right_pose_callback)
        self.left_tactile_sub = rospy.Subscriber("/tomm/arm_right/hand/left_hand_back/data", SkinPatchData, self.left_tactile_callback)
        self.right_tactile_sub = rospy.Subscriber("/tomm/arm_right/hand/right_hand_back/data", SkinPatchData, self.right_tactile_callback)
        self.image_sub = rospy.Subscriber("/usb_camera/republished/compressed", CompressedImage, self.image_callback)

    def reset(self):
        self.left_pose, self.right_pose = None, None
        self.left_proximity, self.right_proximity = None, None
        self.left_image, self.right_image = None, None

#-----------------------------------------------------------------------------------------------------------------------------------------
    def run(self):
        self.reset()
        state = [None, None, None]
        rate = rospy.Rate(self.inference_rate)

        while not rospy.is_shutdown() and \
            (self.left_image is None or \
             self.left_pose is None or self.right_pose is None or \
             self.left_proximity is None or self.right_proximity is None):
            rospy.logwarn_throttle(1.0, 'Waiting for data')
        
        rospy.logwarn('Real-time inference: Start')

        while not rospy.is_shutdown():
            y_left_image, y_right_image, left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, _ = self.inference(state)
            self.visualize(y_left_image, y_right_image, left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts)
            rate.sleep()

#-----------------------------------------------------------------------------------------------------------------------------------------
    def inference(self, state):
        # preprocess input data
        x_image = [self.process_image(self.left_image), self.process_image(self.right_image)]
        x_pose = [self.process_signal(self.left_pose, self.left_pose_bounds),
                   self.process_signal(self.right_pose, self.right_pose_bounds)]
        x_proximity = [self.process_signal(self.left_proximity, self.left_proximity_bounds),
                       self.process_signal(self.right_proximity, self.right_proximity_bounds)]
        
        with torch.no_grad():
            y_image,  y_pose, y_proximity, enc_pts, dec_pts, state = \
                self.model(x_image, x_pose, x_proximity, state)
            
        # denormalization
        y_left_image = self.denorm_image(y_image[0][0])
        y_right_image = self.denorm_image(y_image[1][0])

        _left_enc_pts = tensor2numpy(enc_pts[0][0])
        _left_dec_pts = tensor2numpy(dec_pts[0][0])
        _right_enc_pts = tensor2numpy(enc_pts[1][0])
        _right_dec_pts = tensor2numpy(dec_pts[1][0])

        def _split_key_points(points, image_size=64):
            _points = points.reshape(-1, self.k_dim, 2) * image_size
            _points = np.clip(_points[0], 0, image_size)
            return _points
        left_enc_pts = _split_key_points(_left_enc_pts)
        left_dec_pts = _split_key_points(_left_dec_pts)
        right_enc_pts = _split_key_points(_right_enc_pts)
        right_dec_pts = _split_key_points(_right_dec_pts)

        return y_left_image, y_right_image, left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state


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
        #blank_space = np.zeros((64, 16, 3))

        #result_image = np.concatenate((x_image, blank_space, y_image), axis=1)
        result_image = np.concatenate((x_image, y_image), axis=1)
        cv2.imshow("Image", result_image)
        cv2.waitKey(3)

#-----------------------------------------------------------------------------------------------------------------------------------------
    def process_image(self, image):
        _image = image.transpose(2, 0, 1)
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
    
#-----------------------------------------------------------------------------------------------------------------------------------------
    def left_pose_callback(self, msg):
        self.left_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def right_pose_callback(self, msg):
        self.right_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def left_tactile_callback(self, msg):
        self.left_proximity = msg.prox

    def right_tactile_callback(self, msg):
        self.right_proximity = msg.prox
    
    def image_callback(self, msg):
        crop_x1 = int(100/2)
        crop_y1 = int(114/2)
        crop_size = int(320/2)
        np_arr = np.frombuffer(msg.data, np.uint8)
        np_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        np_img = np_img[::2, ::2]
        _left_image = np_img[crop_y1 : crop_y1+crop_size, crop_x1 : crop_x1+crop_size].astype(np.uint8)
        _right_image = np_img[crop_y1 : crop_y1+crop_size, 320-crop_size : 320].astype(np.uint8)
        self.left_image = cv2.resize(_left_image, (64, 64))
        self.right_image = cv2.resize(_right_image, (64, 64))

#-----------------------------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node("online_SARNN_image_node")
    rospy.loginfo("online_SARNN_image_node start")
    rospy.sleep(1.0)

    SARNN_image = OnlineSARNNImage(
        data_dir="./log/k_dim_5", 
        model_name="20231121_0409_26", 
        inference_rate=10)
    SARNN_image.run()

    rospy.loginfo("online_SARNN_image_node shutdown")

if __name__ == "__main__":
    main()