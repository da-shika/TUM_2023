#! /usr/bin/env python3

import math
import numpy as np
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import TransformStamped

PI = math.pi

class VivePR2Controller:
    def __init__(self):
        rospy.init_node("vive_pr2_controller")
        rospy.loginfo("vive_pr2_controller_node start")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.config()
        self.config_position()
        self.config_rotation()
        self.config_marix()

        self.right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.Joycallback)
        self.send_cmd_pub = rospy.Publisher("/robot_IO_cmd", String, queue_size=10)
        self.pr2_left_target_pos_pub = rospy.Publisher("/pr2_left_target_pos", Float32MultiArray, queue_size=10)
        self.pr2_right_target_pos_pub = rospy.Publisher("/pr2_right_target_pos", Float32MultiArray, queue_size=10)
        self.pr2_left_target_rot_pub = rospy.Publisher("/pr2_left_target_rot", Float32MultiArray, queue_size=10)
        self.pr2_right_target_rot_pub = rospy.Publisher("/pr2_right_target_rot", Float32MultiArray, queue_size=10)

    def config(self):
        self.is_robot_on = False
        self.right_button = None
        self.robot_cmd = None
    
    def config_position(self):
        self.vive_left_reference_pos = None
        self.vive_right_reference_pos = None
        self.pr2_left_reference_pos = None
        self.pr2_right_reference_pos = None

        self.vive_left_relative_pos = None
        self.vive_right_relative_pos = None
        self.pr2_left_relative_pos = None
        self.pr2_right_relative_pos = None

        self.pr2_left_target_pos = None
        self.pr2_right_target_pos = None

    def config_rotation(self):
        self.vive_left_current_rot = None
        self.vive_right_current_rot = None
        self.pr2_left_wrist_target_rot = None
        self.pr2_right_wrist_target_rot = None

    def config_marix(self):
        def quaternion_to_rot_mat(quaternion):
            return Rotation.from_quat(quaternion).as_matrix()
        def rot_mat_to_quaternion(rot_mat):
            return Rotation.from_matrix(rot_mat).as_quat()
        def euler_to_quaternion(euler):
            return Rotation.from_euler("xyz", euler, degrees=False).as_quat()

        self.R_world_to_base = np.array([[0,-1,0],
                                        [1,0,0],
                                        [0,0,1]])
        q_world_to_base = rot_mat_to_quaternion(self.R_world_to_base)

        q_left_wrist_to_link = np.array([-0.0014864, -0.35162, 0.00078016, 0.93614])
        q_left_vive_to_world = np.array([0.04503167541099551, 0.9717776070935632, 0.23153466555570307, 0.0034827011349717156])  #same direction with robot
        q_right_wrist_to_link = np.array([0.00057509, -0.35149, -0.00039639, 0.93619])
        q_right_vive_to_world = np.array([0.09690744189581683, 0.9550075977364274, 0.27434072359528616, 0.05750307251027345])    #same direction with robot
        
        R_left_wrist_to_link = quaternion_to_rot_mat(q_left_wrist_to_link)
        R_left_vive_to_world = quaternion_to_rot_mat(q_left_vive_to_world)
        R_right_wrist_to_link = quaternion_to_rot_mat(q_right_wrist_to_link)
        R_right_vive_to_world = quaternion_to_rot_mat(q_right_vive_to_world)

        R_left_vive_to_wrist = np.dot(np.dot(np.transpose(R_left_wrist_to_link), self.R_world_to_base), R_left_vive_to_world)
        R_right_vive_to_wrist = np.dot(np.dot(np.transpose(R_right_wrist_to_link), self.R_world_to_base), R_right_vive_to_world)

        self.q_left_vive_to_wrist = rot_mat_to_quaternion(R_left_vive_to_wrist)
        self.q_right_vive_to_wrist = rot_mat_to_quaternion(R_right_vive_to_wrist)

    def Joycallback(self, msg):
        self.right_button = msg.buttons[0]

#-------------------------------------------------------------------------------------------------------------------
    def send_cmd (self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if self.right_button == 0:
                self.robot_cmd = "OFF"
                if self.is_robot_on == False:
                    pass
                elif self.is_robot_on == True:
                    rospy.loginfo("robot switch off")
                    self.is_robot_on = False
                    self.send_cmd_pub.publish(self.robot_cmd)
            
            elif self.right_button == 1:
                self.robot_cmd = "ON"
                if self.is_robot_on == False:
                    rospy.loginfo("robot switch on")
                    self.is_robot_on = True
                    self.send_cmd_pub.publish(self.robot_cmd)
                    self.set_reference()
                elif self.is_robot_on == True:
                    pass
            
            if self.is_robot_on == True:
                if self.vive_left_reference_pos is not None and self.vive_right_reference_pos is not None and self.pr2_right_reference_pos is not None and self.pr2_right_reference_pos is not None:
                    self.update_relative_position()
            
            rate.sleep()

#-------------------------------------------------------------------------------------------------------------------
    def set_reference(self):
        # get initial position and rotation
        self.vive_left_reference_pos, _ = self.get_trans_rot_from_tf("world", "controller_LHR_9F5D6499")
        self.vive_right_reference_pos, _ = self.get_trans_rot_from_tf("world", "controller_LHR_5920DD6C")
        self.pr2_left_reference_pos, _ = self.get_trans_rot_from_tf("base_link", "l_wrist_roll_link")
        self.pr2_right_reference_pos, _ = self.get_trans_rot_from_tf("base_link", "r_wrist_roll_link")

        rospy.loginfo("\033[32mSuccess to set reference position!\033[0m")


    def update_relative_position(self):
        # get relative position of vive
        def get_vive_relative_pos(current, reference):
            relative_0 = current[0] - reference[0]
            relative_1 = current[1] - reference[1]
            relative_2 = current[2] - reference[2]
            return (relative_0, relative_1, relative_2)

        vive_left_current_translation, self.vive_left_current_rot = self.get_trans_rot_from_tf("world", "controller_LHR_9F5D6499")
        self.vive_left_relative_pos = get_vive_relative_pos(vive_left_current_translation, self.vive_left_reference_pos)

        vive_right_current_translation, self.vive_right_current_rot = self.get_trans_rot_from_tf("world", "controller_LHR_5920DD6C")
        self.vive_right_relative_pos = get_vive_relative_pos(vive_right_current_translation, self.vive_right_reference_pos)

        # get relative position of pr2
        self.pr2_left_relative_pos = np.dot(self.R_world_to_base, self.vive_left_relative_pos)
        self.pr2_right_relative_pos = np.dot(self.R_world_to_base, self.vive_right_relative_pos)

        # get pr2 target positions
        def get_pr2_target_pos(reference, relative):
            target_0 = reference[0] + relative[0]
            target_1 = reference[1] + relative[1]
            target_2 = reference[2] + relative[2]
            return (target_0, target_1, target_2)
        
        def get_pr2_wrist_target_rot(vive_to_wrist, vive_to_world):
            target_rot = self.quaternion_multiply(vive_to_wrist, vive_to_world)
            target_rot /= np.linalg.norm(target_rot)
            return target_rot
        
        self.pr2_left_target_pos = get_pr2_target_pos(self.pr2_left_reference_pos, self.pr2_left_relative_pos)
        self.pr2_right_target_pos = get_pr2_target_pos(self.pr2_right_reference_pos, self.pr2_right_relative_pos)

        self.pr2_left_wrist_target_rot = get_pr2_wrist_target_rot(self.q_left_vive_to_wrist, self.vive_left_current_rot)
        self.pr2_right_wrist_target_rot = get_pr2_wrist_target_rot(self.q_right_vive_to_wrist, self.vive_right_current_rot)

        # visualize pr2 target
        self.set_tf_broadcast("base_link", "pr2_left_target_tf", self.pr2_left_target_pos, self.pr2_left_wrist_target_rot)
        self.set_tf_broadcast("base_link", "pr2_right_target_tf", self.pr2_right_target_pos, self.pr2_right_wrist_target_rot)

        # publish target pos
        def publish(data, publisher):
            msg = Float32MultiArray()
            msg.data = data
            publisher.publish(msg)

        publish(list(self.pr2_left_target_pos), self.pr2_left_target_pos_pub)
        publish(list(self.pr2_right_target_pos), self.pr2_right_target_pos_pub)
        publish(list(self.pr2_left_wrist_target_rot), self.pr2_left_target_rot_pub)
        publish(list(self.pr2_right_wrist_target_rot), self.pr2_right_target_rot_pub)

#-------------------------------------------------------------------------------------------------------------------
    def get_trans_rot_from_tf(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            pos = (translation.x, translation.y, translation.z)
            rot = (rotation.x, rotation.y, rotation.z, rotation.w)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF transform lookup failed in set reference")
        return pos, rot
    

    def set_tf_broadcast(self, base_frame, child_frame, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = base_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(transform)

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([x, y, z, w])
    
#-------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        vive_pr2_controller = VivePR2Controller()
        vive_pr2_controller.send_cmd()
    except rospy.ROSInitException:
        pass