#! /usr/bin/env python

import os
import sys
import glob
import datetime
import numpy as np
import cv2
import torch
import rospy
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import PoseStamped
from skin_client.msg import SkinPatchData
from behavior_msgs.srv import ChangeBehavior, ChangeBehaviorRequest, ListBehavior, ListBehaviorRequest
from eipl.utils import normalization, deprocess_img, tensor2numpy, restore_args

try:
    from libs.model import MTRNN
except:
    sys.path.append("./libs/")
    from model import MTRNN

def is_teleop_running(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/list_behavior', ListBehavior)
    req = srv_client.call(ListBehaviorRequest())
    return name in req.running_behaviors

def disable_teleop(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/change_behavior', ChangeBehavior)
    req = ChangeBehaviorRequest()
    req.start_behaviors = []
    req.stop_behaviors = [name]
    srv_client.call(req)

def enable_teleop(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/change_behavior', ChangeBehavior)
    req = ChangeBehaviorRequest()
    req.start_behaviors = [name]
    req.stop_behaviors = []
    srv_client.call(req)


class OnlineMTRNN:
    def __init__(self, data_dir, model_name, inference_rate=10, max_episode_steps=182):
        if not is_teleop_running():
            enable_teleop()
        self.model_name = model_name
        self.save_dir = os.path.join(data_dir, model_name)
        self.inference_rate = inference_rate
        self.max_episode_steps = max_episode_steps
        self.warmup_steps = 5
        
        # hard code orientation
        self.left_orientation = [-0.499, 0.498, 0.482, 0.519]
        self.right_orientation = [0.490, 0.510, -0.493, 0.507]

        USE_CUDA = -1
        params = restore_args( os.path.join(self.save_dir, 'args.json') )
        self.params = params

        # load weight
        ckpt_file = sorted(glob.glob(os.path.join(self.save_dir, '*.pth')))
        latest = ckpt_file.pop()

        # load bounds
        data_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/re_no_rotate/data/dual" + "/"
        self.minmax = [params["vmin"], params["vmax"]]
        self.left_pose_bounds = np.load(data_dir + "left_pose_bounds.npy")
        self.left_force_bounds = np.load(data_dir + "left_force_bounds.npy")
        self.left_proximity_bounds = np.load(data_dir + "left_proximity_bounds.npy")
        self.right_pose_bounds = np.load(data_dir + "right_pose_bounds.npy")
        self.right_force_bounds = np.load(data_dir + "right_force_bounds.npy")
        self.right_proximity_bounds = np.load(data_dir + "right_proximity_bounds.npy")

        self.k_dim = params["k_dim"]
        self.prev_dec_pts = None

        # define model
        self.model = MTRNN(
            fast_dim=params["fast_dim"], slow_dim=params["slow_dim"], union_dim=params["union_dim"],
            fast_tau=params["fast_tau"], slow_tau=params["slow_tau"], union_tau=params["union_tau"],
            pose_dim=3,
            force_dim=1,
            proximity_dim=1,
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
        self.left_tactile_sub = rospy.Subscriber("/tomm/arm_right/hand/left_hand_back/data", SkinPatchData, self.left_tactile_callback)
        self.right_tactile_sub = rospy.Subscriber("/tomm/arm_right/hand/right_hand_back/data", SkinPatchData, self.right_tactile_callback)
        self.image_sub = rospy.Subscriber("/usb_camera/republished/compressed", CompressedImage, self.image_callback)

        self.target_left_pos_pub = rospy.Publisher("/target/left_pose", PoseStamped, queue_size=1)
        self.target_right_pos_pub = rospy.Publisher("/target/right_pose", PoseStamped, queue_size=1)

    def reset(self):
        self.left_pose, self.right_pose = None, None
        self.left_force, self.right_force = None, None
        self.left_proximity, self.right_proximity = None, None
        self.left_image, self.right_image = None, None
        self.steps = 0

        self.left_image_raw, self.right_image_raw = [], []
        self.left_image_dec, self.right_image_dec = [], []
        self.left_enc_pts, self.right_enc_pts = [], []
        self.left_dec_pts, self.right_dec_pts = [], []
        self.pred_left_pose, self.pred_right_pose = [], []
        self.pred_left_force, self.pred_right_force = [], []
        self.pred_left_prox, self.pred_right_prox = [], []
        self.real_left_pose, self.real_right_pose = [], []
        self.real_left_force, self.real_right_force = [], []
        self.real_left_prox, self.real_right_prox = [], []
        self.left_pose_h_fast, self.left_enc_h_fast, self.left_force_h_fast, self.left_prox_h_fast, self.left_h_slow = [], [], [], [], []
        self.left_pose_u_fast, self.left_enc_u_fast, self.left_force_u_fast, self.left_prox_u_fast, self.left_u_slow = [], [], [], [], []
        self.right_pose_h_fast, self.right_enc_h_fast, self.right_force_h_fast, self.right_prox_h_fast, self.right_h_slow = [], [], [], [], []
        self.right_pose_u_fast, self.right_enc_u_fast, self.right_force_u_fast, self.right_prox_u_fast, self.right_u_slow = [], [], [], [], []
        self.h_union, self.u_union = [], []
        self.left_modality_att, self.right_modality_att = [], []

#-----------------------------------------------------------------------------------------------------------------------------------------
    def run(self):
        self.reset()
        state = [None, None, None]
        rate = rospy.Rate(self.inference_rate)

        while not rospy.is_shutdown() and \
            (self.left_image is None or \
             self.left_pose is None or self.right_pose is None or \
             self.left_force is None or self.right_force is None or \
             self.left_proximity is None or self.right_proximity is None):
            rospy.logwarn_throttle(1.0, 'Waiting for data')
        
        rospy.logwarn('Real-time inference: Start')

        while not rospy.is_shutdown() and self.steps < self.max_episode_steps:
            y_left_image, y_right_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_proximity, y_right_proximity, \
                left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state, modality_att = self.inference(state)

            # send to robot
            if self.steps > self.warmup_steps:
                self.publish(y_left_pose, self.left_orientation, self.left_target_pose_msg, self.target_left_pos_pub)
                self.publish(y_right_pose, self.right_orientation, self.right_target_pose_msg, self.target_right_pos_pub)
            self.visualize(y_left_image, y_right_image, left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts)
            
            # append the params for save
            self.append_params(y_left_image, y_right_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_proximity, y_right_proximity, \
                                left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state, modality_att)

            self.steps += 1  
            rate.sleep()

        while True:
            def _initialize():
                user_input_2 = input("i/h:")
                if user_input_2 == "i" and not is_teleop_running():
                    enable_teleop()
                elif user_input_2 == "h" and is_teleop_running():
                    disable_teleop()

            user_input = input("y/n:")
            if user_input == "y":
                self.save()
                _initialize()
                break
            elif user_input == "n":
                _initialize()
                break
            else:
                print("y/n:")

#-----------------------------------------------------------------------------------------------------------------------------------------
    def inference(self, state):
        # preprocess input data
        x_image = [self.process_image(self.left_image), self.process_image(self.right_image)]
        x_pose = [self.process_pose(self.left_pose, self.left_pose_bounds), 
                  self.process_pose(self.right_pose, self.right_pose_bounds)]
        x_force = [self.process_tactile(self.left_force, self.left_force_bounds), 
                   self.process_tactile(self.right_force, self.right_force_bounds)]
        x_proximity = [self.process_tactile(self.left_proximity, self.left_proximity_bounds), 
                       self.process_tactile(self.right_proximity, self.right_proximity_bounds)]
        
        with torch.no_grad():
            y_image, y_pose, y_force, y_proximity, enc_pts, dec_pts, state, modality_att = self.model(x_image, x_pose, x_force, x_proximity, state)
            
        # denormalization
        y_left_image = self.denorm_image(y_image[0][0])
        y_right_image = self.denorm_image(y_image[1][0])

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

        return y_left_image, y_right_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_proximity, y_right_proximity, \
                left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state, modality_att


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

    def append_params(self, y_left_image, y_right_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_proximity, y_right_proximity, \
                    left_enc_pts, left_dec_pts, right_enc_pts, right_dec_pts, state, modality_att):
        self.left_image_raw.append(self.left_image)
        self.right_image_raw.append(self.right_image)
        self.left_image_dec.append(y_left_image)
        self.right_image_dec.append(y_right_image)
        self.left_enc_pts.append(left_enc_pts)
        self.right_enc_pts.append(right_enc_pts)
        self.left_dec_pts.append(left_dec_pts)
        self.right_dec_pts.append(right_dec_pts)
        
        self.pred_left_pose.append(y_left_pose)
        self.pred_right_pose.append(y_right_pose)
        self.pred_left_force.append(y_left_force)
        self.pred_right_force.append(y_right_force)
        self.pred_left_prox.append(y_left_proximity)
        self.pred_right_prox.append(y_right_proximity)
        
        self.real_left_pose.append(self.left_pose)
        self.real_right_pose.append(self.right_pose)
        self.real_left_force.append(max(self.left_force))
        self.real_right_force.append(max(self.right_force))
        self.real_left_prox.append(max(self.left_proximity))
        self.real_right_prox.append(max(self.right_proximity))
        
        self.left_pose_h_fast.append(tensor2numpy(state[0][0]))
        self.left_enc_h_fast.append(tensor2numpy(state[0][1]))
        self.left_force_h_fast.append(tensor2numpy(state[0][2]))
        self.left_prox_h_fast.append(tensor2numpy(state[0][3]))
        self.left_h_slow.append(tensor2numpy(state[0][4]))
        self.left_pose_u_fast.append(tensor2numpy(state[0][5]))
        self.left_enc_u_fast.append(tensor2numpy(state[0][6]))
        self.left_force_u_fast.append(tensor2numpy(state[0][7]))
        self.left_prox_u_fast.append(tensor2numpy(state[0][8]))
        self.left_u_slow.append(tensor2numpy(state[0][9]))

        self.right_pose_h_fast.append(tensor2numpy(state[1][0]))
        self.right_enc_h_fast.append(tensor2numpy(state[1][1]))
        self.right_force_h_fast.append(tensor2numpy(state[1][2]))
        self.right_prox_h_fast.append(tensor2numpy(state[1][3]))
        self.right_h_slow.append(tensor2numpy(state[1][4]))
        self.right_pose_u_fast.append(tensor2numpy(state[1][5]))
        self.right_enc_u_fast.append(tensor2numpy(state[1][6]))
        self.right_force_u_fast.append(tensor2numpy(state[1][7]))
        self.right_prox_u_fast.append(tensor2numpy(state[1][8]))
        self.right_u_slow.append(tensor2numpy(state[1][9]))

        self.h_union.append(tensor2numpy(state[2][0]))
        self.u_union.append(tensor2numpy(state[2][1]))

        self.left_modality_att.append(tensor2numpy(modality_att[0]))
        self.right_modality_att.append(tensor2numpy(modality_att[1]))


    def save(self):
        left_image = np.array(self.left_image_raw)
        right_image = np.array(self.right_image_raw)
        left_image_dec = np.array(self.left_image_dec)
        right_image_dec = np.array(self.right_image_dec)
        left_enc_pts = np.array(self.left_enc_pts)
        right_enc_pts = np.array(self.right_enc_pts)
        left_dec_pts = np.array(self.left_dec_pts)
        right_dec_pts = np.array(self.right_dec_pts)

        prev_left_pose = np.array(self.pred_left_pose)
        prev_right_pose = np.array(self.pred_right_pose)
        prev_left_force = np.array(self.pred_left_force)
        prev_right_force = np.array(self.pred_right_force)
        prev_left_prox = np.array(self.pred_left_prox)
        prev_right_prox = np.array(self.pred_right_prox)

        real_left_pose = np.array(self.real_left_pose)
        real_right_pose = np.array(self.real_right_pose)
        real_left_force = np.array(self.real_left_force)
        real_right_force = np.array(self.real_right_force)
        real_left_prox = np.array(self.real_left_prox)
        real_right_prox = np.array(self.real_right_prox)

        left_pose_h_fast = np.array(self.left_pose_h_fast)
        left_enc_h_fast = np.array(self.left_enc_h_fast)
        left_force_h_fast = np.array(self.left_force_h_fast)
        left_prox_h_fast = np.array(self.left_prox_h_fast)
        left_h_slow = np.array(self.left_h_slow)
        left_pose_u_fast = np.array(self.left_pose_u_fast)
        left_enc_u_fast = np.array(self.left_enc_u_fast)
        left_force_u_fast = np.array(self.left_force_u_fast)
        left_prox_u_fast = np.array(self.left_prox_u_fast)
        left_u_slow = np.array(self.left_u_slow)

        right_pose_h_fast = np.array(self.right_pose_h_fast)
        right_enc_h_fast = np.array(self.right_enc_h_fast)
        right_force_h_fast = np.array(self.right_force_h_fast)
        right_prox_h_fast = np.array(self.right_prox_h_fast)
        right_h_slow = np.array(self.right_h_slow)
        right_pose_u_fast = np.array(self.right_pose_u_fast)
        right_enc_u_fast = np.array(self.right_enc_u_fast)
        right_force_u_fast = np.array(self.right_force_u_fast)
        right_prox_u_fast = np.array(self.right_prox_u_fast)
        right_u_slow = np.array(self.right_u_slow)

        h_union = np.array(self.h_union)
        u_union = np.array(self.u_union)

        left_modality_att = np.array(self.left_modality_att)
        right_modality_att = np.array(self.right_modality_att)
        
        save_dir = os.path.join("./realtime_motion_output", self.model_name)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        user_input = input("object_(success or failure):")
        np.savez(save_dir +"/"+ datetime.datetime.today().strftime("%Y%m%d_%H%M_%S_") + user_input, 
                 left_image=left_image, right_image=right_image, left_image_dec=left_image_dec, right_image_dec=right_image_dec,
                 left_enc_pts=left_enc_pts, right_enc_pts=right_enc_pts, left_dec_pts=left_dec_pts, right_dec_pts=right_dec_pts,
                 prev_left_pose=prev_left_pose, prev_right_pose=prev_right_pose, 
                 prev_left_force=prev_left_force, prev_right_force=prev_right_force, 
                 prev_left_prox=prev_left_prox, prev_right_prox=prev_right_prox,
                 real_left_pose=real_left_pose, real_right_pose=real_right_pose, 
                 real_left_force=real_left_force, real_right_force=real_right_force, 
                 real_left_prox=real_left_prox, real_right_prox=real_right_prox,
                 left_pose_h_fast=left_pose_h_fast, left_enc_h_fast=left_enc_h_fast, left_force_h_fast=left_force_h_fast, left_prox_h_fast=left_prox_h_fast, left_h_slow=left_h_slow,
                 left_pose_u_fast=left_pose_u_fast, left_enc_u_fast=left_enc_u_fast, left_force_u_fast=left_force_u_fast, left_prox_u_fast=left_prox_u_fast, left_u_slow=left_u_slow,
                 right_pose_h_fast=right_pose_h_fast, right_enc_h_fast=right_enc_h_fast, right_force_h_fast=right_force_h_fast, right_prox_h_fast=right_prox_h_fast, right_h_slow=right_h_slow,
                 right_pose_u_fast=right_pose_u_fast, right_enc_u_fast=right_enc_u_fast, right_force_u_fast=right_force_u_fast, right_prox_u_fast=right_prox_u_fast, right_u_slow=right_u_slow,
                 h_union=h_union, u_union=u_union,
                 left_modality_att=left_modality_att, right_modality_att=right_modality_att
                 )
        rospy.logwarn("Save the data!")

#-----------------------------------------------------------------------------------------------------------------------------------------
    def process_image(self, image):
        _image = image.transpose(2, 0, 1)
        _image = normalization(_image, (0, 255), self.minmax)
        _image = np.expand_dims(_image, axis=0)
        return torch.from_numpy(_image).float()

    def process_pose(self, signal, bounds):
        _signal = normalization(signal, bounds, self.minmax)
        _signal = np.expand_dims(_signal, axis=0)
        return torch.from_numpy(_signal).float()
    
    def process_tactile(self, signal, bounds):
        _signal_max = max(signal)
        _signal = normalization(_signal_max, bounds, self.minmax)
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

    def left_tactile_callback(self, msg):
        self.left_force = list(msg.force)
        del self.left_force[8]
        self.left_force = tuple(self.left_force)
        self.left_proximity = msg.prox

    def right_tactile_callback(self, msg):
        self.right_force = msg.force
        self.right_proximity = msg.prox
    
    def image_callback(self, msg):
        crop_x1 = int(100/2)
        crop_y1 = int(64/2)
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
    rospy.init_node("online_MTRNN_node")
    rospy.loginfo("online_MTRNN_node start")
    rospy.sleep(1.0)

    MTRNN_motion = OnlineMTRNN(
        data_dir="./log", 
        model_name="20231207_0509_44", 
        inference_rate=10,
        max_episode_steps=182)
    MTRNN_motion.run()

    rospy.loginfo("online_MTRNN_image_node shutdown")

if __name__ == "__main__":
    main()