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

        USE_CUDA = -1
        params = restore_args( os.path.join(self.save_dir, 'args.json') )
        self.params = params

        # load weight
        ckpt_file = sorted(glob.glob(os.path.join(self.save_dir, '*.pth')))
        latest = ckpt_file.pop()

        # load bounds
        data_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/data/dual" + "/"
        self.minmax = [params["vmin"], params["vmax"]]
        self.left_joint_bounds = np.load(data_dir + "left_joint_bounds.npy")
        self.left_pose_bounds = np.load(data_dir + "left_pose_bounds.npy")
        self.left_force_bounds = np.load(data_dir + "left_force_bounds.npy")
        self.left_proximity_bounds = np.load(data_dir + "left_proximity_bounds.npy")
        self.right_joint_bounds = np.load(data_dir + "right_joint_bounds.npy")
        self.right_pose_bounds = np.load(data_dir + "right_pose_bounds.npy")
        self.right_force_bounds = np.load(data_dir + "right_force_bounds.npy")
        self.right_proximity_bounds = np.load(data_dir + "right_proximity_bounds.npy")

        self.k_dim = params["k_dim"]
        self.prev_dec_pts = None

        # define model
        self.model = SARNN(
            rec_dim=params["rec_dim"],
            joint_dim=6,
            pose_dim=7,
            force_dim=10,
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
        self.target_joint_msg = JointState()
        self.target_joint_msg.name = ["l_elbow_joint","l_shoulder_lift_joint","l_shoulder_pan_joint","l_wrist_1_joint","l_wrist_2_joint","l_wrist_3_joint", 
                                    "r_elbow_joint","r_shoulder_lift_joint","r_shoulder_pan_joint","r_wrist_1_joint","r_wrist_2_joint","r_wrist_3_joint"]
        self.reset()

        self.joint_sub = rospy.Subscriber("/tomm/joint_states", JointState, self.joint_callback)
        self.left_pose_sub = rospy.Subscriber("/tomm/teleop_left_hand/pose", PoseStamped, self.left_pose_callback)
        self.right_pose_sub = rospy.Subscriber("/tomm/teleop_right_hand/pose", PoseStamped, self.right_pose_callback)
        self.left_tactile_sub = rospy.Subscriber("/tomm/arm_right/hand/left_hand_back/data", SkinPatchData, self.left_tactile_callback)
        self.right_tactile_sub = rospy.Subscriber("/tomm/arm_right/hand/right_hand_back/data", SkinPatchData, self.right_tactile_callback)
        self.image_sub = rospy.Subscriber("/usb_camera/republished/compressed", CompressedImage, self.image_callback)

        self.target_joint_pub = rospy.Publisher("/target_command", JointState, queue_size=1)

    def reset(self):
        self.left_joint, self.right_joint = None, None
        self.left_pose, self.right_pose = None, None
        self.left_force, self.right_force = None, None
        self.left_proximity, self.right_proximity = None, None
        self.left_image, self.right_image = None, None
        self.steps = 0

#-----------------------------------------------------------------------------------------------------------------------------------------
    def run(self):
        self.reset()
        state = [None, None, None]
        rate = rospy.Rate(self.inference_rate)

        while not rospy.is_shutdown() and \
            (self.left_joint is None or self.left_image is None or \
             self.left_pose is None or self.right_pose is None or \
             self.left_force is None or self.right_force is None):
            rospy.logwarn_throttle(1.0, 'Waiting for data')
        
        rospy.logwarn('Real-time inference: Start')

        while not rospy.is_shutdown() and self.steps < self.max_episode_steps:
            y_left_image, y_right_image, y_left_joint, y_right_joint, y_left_pose, y_right_pose, \
                y_left_force, y_right_force, y_left_proximity, y_right_proximity, \
                left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state = self.inference(state)

            # send to robot
            if self.steps > self.warmup_steps:
                self.publish(y_left_joint, y_right_joint)
            self.visualize(y_left_image, y_right_image, left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts)

            self.steps += 1  
            rate.sleep()

#-----------------------------------------------------------------------------------------------------------------------------------------
    def inference(self, state):
        # preprocess input data
        x_image = [self.process_image(self.left_image), self.process_image(self.right_image)]
        x_joint = [self.process_signal(self.left_joint, self.left_joint_bounds),
                   self.process_signal(self.right_joint, self.right_joint_bounds)]
        x_pose = [self.process_signal(self.left_pose, self.left_pose_bounds),
                   self.process_signal(self.right_pose, self.right_pose_bounds)]
        x_force = [self.process_signal(self.left_force, self.left_force_bounds),
                   self.process_signal(self.right_force, self.right_force_bounds)]
        x_proximity = [self.process_signal(self.left_proximity, self.left_proximity_bounds),
                       self.process_signal(self.right_proximity, self.right_proximity_bounds)]
        
        with torch.no_grad():
            y_image, y_joint, y_pose, y_force, y_proximity, enc_pts, dec_pts, state = \
                self.model(x_image, x_joint, x_pose, x_force, x_proximity, state)
            
        # denormalization
        y_left_image = self.denorm_image(y_image[0][0])
        y_right_image = self.denorm_image(y_image[1][0])

        y_left_joint = self.denorm_signal(y_joint[0][0], self.left_joint_bounds)
        y_right_joint = self.denorm_signal(y_joint[1][0], self.right_joint_bounds)
        y_left_pose = self.denorm_signal(y_pose[0][0], self.left_pose_bounds)
        y_right_pose = self.denorm_signal(y_pose[1][0], self.right_pose_bounds)

        y_left_force = self.denorm_signal(y_force[0][0], self.left_force_bounds)
        y_right_force = self.denorm_signal(y_force[1][0], self.right_force_bounds)
        y_left_proximity = self.denorm_signal(y_proximity[0][0], self.left_proximity_bounds)
        y_right_proximity = self.denorm_signal(y_proximity[1][0], self.right_proximity_bounds)

        left_enc_pts = self.split_key_points(enc_pts[0][0])
        left_dec_pts = self.split_key_points(dec_pts[0][0])
        right_enc_pts = self.split_key_points(enc_pts[1][0])
        right_dec_pts = self.split_key_points(dec_pts[1][0])

        return y_left_image, y_right_image, y_left_joint, y_right_joint, y_left_pose, y_right_pose, \
                y_left_force, y_right_force, y_left_proximity, y_right_proximity, \
                left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state


    def publish(self, y_left_pose, y_right_pose):
        y_pose = np.concatenate((y_left_pose, y_right_pose), axis=0)
        self.target_joint_msg.position = y_pose.tolist()
        self.target_joint_msg.header.stamp = rospy.Time.now()
        self.target_joint_pub.publish(self.target_joint_msg)


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
    def joint_callback(self, msg):
        self.left_joint = msg.position[3:9]
        self.right_joint = msg.position[9:15]

    def left_pose_callback(self, msg):
        _pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        _rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.left_pose = _pos + _rot

    def right_pose_callback(self, msg):
        _pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        _rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.right_pose = _pos + _rot

    def left_tactile_callback(self, msg):
        self.left_force = msg.force
        self.left_proximity = msg.prox

    def right_tactile_callback(self, msg):
        self.right_force = msg.force
        self.right_proximity = msg.prox
    
    def image_callback(self, msg):
        crop_x1 = 0
        crop_y1 = int(55/2)
        crop_size = int(320/2)
        np_arr = np.frombuffer(msg.data, np.uint8)
        np_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        np_img = np_img[::2, ::2]
        _left_image = np_img[crop_y1 : crop_y1+crop_size, crop_x1 : crop_x1+crop_size].astype(np.uint8)
        _right_image = np_img[crop_y1 : crop_y1+crop_size, crop_x1+crop_size : crop_x1+2*crop_size].astype(np.uint8)
        self.left_image = cv2.resize(_left_image, (64, 64))
        self.right_image = cv2.resize(_right_image, (64, 64))

#-----------------------------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node("online_SARNN_image_node")
    rospy.loginfo("online_SARNN_image_node start")
    rospy.sleep(1.0)

    SARNN_motion = OnlineSARNN(
        data_dir="./log/k_dim_5", 
        model_name="20231103_0008_59", 
        inference_rate=10,
        max_episode_steps=270)
    SARNN_motion.run()

    rospy.loginfo("online_SARNN_image_node shutdown")

if __name__ == "__main__":
    main()