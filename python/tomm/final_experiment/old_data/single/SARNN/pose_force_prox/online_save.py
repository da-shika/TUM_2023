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
    from libs.model import SARNN
except:
    sys.path.append("./libs/")
    from model import SARNN

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


class OnlineSARNN:
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
        data_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/re_no_rotate/data/single" + "/"
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
        self.model = SARNN(
            rec_dim=params["rec_dim"],
            pose_dim=6,
            force_dim=2,
            prox_dim=2,
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
        self.image = None
        self.steps = 0

        self.image_raw = []
        self.image_dec = []
        self.enc_pts = []
        self.dec_pts = []
        self.pred_left_pose, self.pred_right_pose = [], []
        self.pred_left_force, self.pred_right_force = [], []
        self.pred_left_prox, self.pred_right_prox = [], []
        self.real_left_pose, self.real_right_pose = [], []
        self.real_left_force, self.real_right_force = [], []
        self.real_left_prox, self.real_right_prox = [], []
        self.h_state_list = []
        self.c_state_list = []

#-----------------------------------------------------------------------------------------------------------------------------------------
    def run(self):
        self.reset()
        state = None
        rate = rospy.Rate(self.inference_rate)

        while not rospy.is_shutdown() and \
            (self.image is None or \
             self.left_pose is None or self.right_pose is None or \
             self.left_force is None or self.right_force is None or \
             self.left_proximity is None or self.right_proximity is None):
            rospy.logwarn_throttle(1.0, 'Waiting for data')
        
        rospy.logwarn('Real-time inference: Start')

        while not rospy.is_shutdown() and self.steps < self.max_episode_steps:
            y_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_prox, y_right_prox, enc_pts, dec_pts, state = self.inference(state)

            # send to robot
            if self.steps > self.warmup_steps:
                self.publish(y_left_pose, self.left_orientation, self.left_target_pose_msg, self.target_left_pos_pub)
                self.publish(y_right_pose, self.right_orientation, self.right_target_pose_msg, self.target_right_pos_pub)
            self.visualize(y_image, enc_pts, dec_pts)

            # append the params for save
            self.append_params(y_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_prox, y_right_prox, enc_pts, dec_pts, state)

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
        x_image = self.process_image(self.image)
        x_pose = self.process_pose(self.left_pose, self.left_pose_bounds, self.right_pose, self.right_pose_bounds)
        x_force = self.process_tactile(self.left_force, self.left_force_bounds, self.right_force, self.right_force_bounds)
        x_prox = self.process_tactile(self.left_proximity, self.left_proximity_bounds, self.right_proximity, self.right_proximity_bounds)
        
        with torch.no_grad():
            y_image, y_pose, y_force, y_prox, enc_pts, dec_pts, state = self.model(x_image, x_pose, x_force, x_prox, state)
            
        # denormalization
        y_image = self.denorm_image(y_image[0])
        y_left_pose, y_right_pose = self.denorm_signal(y_pose, self.left_pose_bounds, self.right_pose_bounds)
        y_left_force, y_right_force = self.denorm_signal(y_force, self.left_force_bounds, self.right_force_bounds)
        y_left_prox, y_right_prox = self.denorm_signal(y_prox, self.left_proximity_bounds, self.right_proximity_bounds)
        enc_pts = self.split_key_points(enc_pts[0])
        dec_pts = self.split_key_points(dec_pts[0])

        return y_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_prox, y_right_prox, enc_pts, dec_pts, state


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


    def visualize(self, y_image, enc_pts, dec_pts):
        fac = 7
        x_image = cv2.resize(self.image, (fac*self.image.shape[1], fac*self.image.shape[0]))
        y_image = cv2.resize(y_image, (fac*y_image.shape[1], fac*y_image.shape[0]))
        enc_pts = enc_pts * fac
        dec_pts = dec_pts * fac

        for i in range(self.k_dim):
            cv2.circle(x_image, dec_pts[i].astype(np.int32), 2, (0,0,255), 5)      # RED
            cv2.circle(x_image, enc_pts[i].astype(np.int32), 2, (0,255,0), 5)      # GREEN

        result_image = np.concatenate((x_image, y_image), axis=1)
        cv2.imshow("Image", result_image)
        cv2.waitKey(3)


    def append_params(self, y_image, y_left_pose, y_right_pose, y_left_force, y_right_force, y_left_prox, y_right_prox, enc_pts, dec_pts, state):
        self.image_raw.append(self.image)
        self.image_dec.append(y_image)
        self.enc_pts.append(enc_pts)
        self.dec_pts.append(dec_pts)
        
        self.pred_left_pose.append(y_left_pose)
        self.pred_right_pose.append(y_right_pose)
        self.pred_left_force.append(y_left_force)
        self.pred_right_force.append(y_right_force)
        self.pred_left_prox.append(y_left_prox)
        self.pred_right_prox.append(y_right_prox)
        
        self.real_left_pose.append(self.left_pose)
        self.real_right_pose.append(self.right_pose)
        self.real_left_force.append(max(self.left_force))
        self.real_right_force.append(max(self.right_force))
        self.real_left_prox.append(max(self.left_proximity))
        self.real_right_prox.append(max(self.right_proximity))

        self.h_state_list.append(tensor2numpy(state[0]))
        self.c_state_list.append(tensor2numpy(state[1]))


    def save(self):
        image = np.array(self.image_raw)
        image_dec = np.array(self.image_dec)
        enc_pts = np.array(self.enc_pts)
        dec_pts = np.array(self.dec_pts)

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

        h_state = np.array(self.h_state_list)
        c_state = np.array(self.c_state_list)
        
        save_dir = os.path.join("./realtime_motion_output", self.model_name)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        user_input = input("object_(success or failure):")
        np.savez(save_dir +"/"+ datetime.datetime.today().strftime("%Y%m%d_%H%M_%S_") + user_input, 
                 image=image, image_dec=image_dec,
                 enc_pts=enc_pts, dec_pts=dec_pts,
                 prev_left_pose=prev_left_pose, prev_right_pose=prev_right_pose, 
                 prev_left_force=prev_left_force, prev_right_force=prev_right_force, 
                 prev_left_prox=prev_left_prox, prev_right_prox=prev_right_prox,
                 real_left_pose=real_left_pose, real_right_pose=real_right_pose, 
                 real_left_force=real_left_force, real_right_force=real_right_force, 
                 real_left_prox=real_left_prox, real_right_prox=real_right_prox,
                 h_state=h_state, c_state=c_state
                 )
        rospy.logwarn("Save the data!")

#-----------------------------------------------------------------------------------------------------------------------------------------
    def process_image(self, image):
        _image = image.transpose(2, 0, 1)
        _image = normalization(_image, (0, 255), self.minmax)
        _image = np.expand_dims(_image, axis=0)
        return torch.from_numpy(_image).float()

    def process_pose(self, left_signal, left_bounds, right_signal, right_bounds):
        _left_signal = normalization(left_signal, left_bounds, self.minmax)
        left_signal = np.expand_dims(_left_signal, axis=0)
        _right_signal = normalization(right_signal, right_bounds, self.minmax)
        right_signal = np.expand_dims(_right_signal, axis=0)
        concat_signal = np.concatenate((left_signal, right_signal), axis=1)
        return torch.from_numpy(concat_signal).float()
    
    def process_tactile(self, left_signal, left_bounds, right_signal, right_bounds):
        _left_signal_max = max(left_signal)
        _left_signal = normalization(_left_signal_max, left_bounds, self.minmax)
        left_signal = np.expand_dims(_left_signal, axis=0)
        _right_signal_max = max(right_signal)
        _right_signal = normalization(_right_signal_max, right_bounds, self.minmax)
        right_signal = np.expand_dims(_right_signal, axis=0)
        concat_signal = np.concatenate((left_signal, right_signal), axis=1)
        return torch.from_numpy(concat_signal).float()
    
    def denorm_image(self, image):
        _image = tensor2numpy(image)
        _image = deprocess_img(_image, self.params["vmin"], self.params["vmax"])
        _image = _image.transpose(1, 2, 0)
        return _image
    
    def denorm_signal(self, signal, left_bounds, right_bounds):
        _pred_signal = np.squeeze(tensor2numpy(signal))
        _left_pred = _pred_signal[: int(len(_pred_signal)/2)]
        left_pred = normalization(_left_pred, self.minmax, left_bounds)
        _right_pred = _pred_signal[int(len(_pred_signal)/2) :]
        right_pred = normalization(_right_pred, self.minmax, right_bounds)
        return left_pred, right_pred
    
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
        crop_x1 = int(114/2)
        crop_y1 = 0
        crop_size = int(480/2)
        np_arr = np.frombuffer(msg.data, np.uint8)
        np_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        np_img = np_img[::2, ::2]
        _image = np_img[crop_y1 : crop_y1+crop_size, crop_x1 : crop_x1+crop_size].astype(np.uint8)
        self.image = cv2.resize(_image, (64, 64))

#-----------------------------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node("online_SARNN_node")
    rospy.loginfo("online_SARNN_node start")
    rospy.sleep(1.0)

    SARNN_motion = OnlineSARNN(
        data_dir="./log/k_dim_10", 
        model_name="20231208_0404_27", 
        inference_rate=10,
        max_episode_steps=182)
    SARNN_motion.run()

    rospy.loginfo("online_SARNN_image_node shutdown")

if __name__ == "__main__":
    main()