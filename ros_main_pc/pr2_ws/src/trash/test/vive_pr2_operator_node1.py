#! /usr/bin/env python3

import math
import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32MultiArray
from scipy.spatial.transform import Rotation
import tf.transformations as tft

"""
vive operation PR2
"""

class ViveToPR2Converter:
    def __init__(self):
        rospy.init_node("vive_to_pr2_converter")
        rospy.loginfo("vive_to_pr2_converter_node start")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.config()
        self.config_position()
        self.config_matrix()

        self.right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.Joycallback)
        self.send_cmd_pub = rospy.Publisher("/robot_IO_cmd", String, queue_size=10)
        self.pr2_left_target_pub = rospy.Publisher("/pr2_left_target_pose", Float32MultiArray, queue_size=10)
        self.pr2_right_target_pub = rospy.Publisher("/pr2_right_target_pose", Float32MultiArray, queue_size=10)


    def config(self):
        self.is_robotswhich = False
        self.right_button = None
        self.robot_cmd = None
        
    def config_position(self):
        self.vive_left_reference_position = None
        self.vive_right_reference_position = None
        self.pr2_left_reference_position = None
        self.pr2_right_reference_position = None

        self.vive_left_relative_position = None
        self.vive_right_relative_position = None
        self.pr2_left_relative_position = None
        self.pr2_right_relative_position = None

        self.pr2_left_target_position = None
        self.pr2_right_target_position = None

    def config_matrix(self):
        self.vive_left_reference_trans_mat = None
        self.vive_right_reference_trans_mat = None
        self.pr2_left_reference_trans_mat = None
        self.pr2_right_reference_trans_mat = None

        self.vive_left_now_trans_mat = None
        self.vive_right_now_trans_mat = None
        self.pr2_left_now_trans_mat = None
        self.pr2_right_now_trans_mat = None

    def Joycallback(self, msg):
        self.right_button = msg.buttons[0]

#-----------------------------------------------------------------------
    def send_cmd(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if self.right_button == 0:
                self.robot_cmd = "OFF"
                if self.is_robotswhich == False:
                    pass
                elif self.is_robotswhich == True:
                    rospy.loginfo("robot switch off")
                    self.is_robotswhich = False
                    self.send_cmd_pub.publish(self.robot_cmd)

            elif self.right_button == 1:
                self.robot_cmd = "ON"
                if self.is_robotswhich == False:
                    rospy.loginfo("robot switch on")
                    self.is_robotswhich = True
                    self.send_cmd_pub.publish(self.robot_cmd)
                    self.set_reference_position()
                elif self.is_robotswhich == True:
                    pass

            if self.is_robotswhich == True:
                if (self.vive_left_reference_position is not None and self.vive_right_reference_position is not None \
                    and self.pr2_right_reference_position is not None and self.pr2_right_reference_position is not None):
                    self.update_relative_position()

            rate.sleep()

#-------------------------------------------------------------------------------------------------------------------
    def set_reference_position(self):
        # get initial position
        try:
            vive_left_transform = self.tf_buffer.lookup_transform("world", "controller_LHR_9F5D6499", rospy.Time())
            vive_left_translation = vive_left_transform.transform.translation
            self.vive_left_reference_position = (vive_left_translation.x, vive_left_translation.y, vive_left_translation.z)
            self.vive_left_reference_trans_mat = self.make_transform_matrix("world", "controller_LHR_9F5D6499")

            vive_right_transform = self.tf_buffer.lookup_transform("world", "controller_LHR_5920DD6C", rospy.Time())
            vive_right_translation = vive_right_transform.transform.translation
            self.vive_right_reference_position = (vive_right_translation.x, vive_right_translation.y, vive_right_translation.z)
            self.vive_right_reference_trans_mat = self.make_transform_matrix("world", "controller_LHR_5920DD6C")

            pr2_left_transform = self.tf_buffer.lookup_transform("base_link", "l_wrist_roll_link", rospy.Time())
            pr2_left_translation = pr2_left_transform.transform.translation
            self.pr2_left_reference_position = (pr2_left_translation.x, pr2_left_translation.y, pr2_left_translation.z)
            self.pr2_left_reference_trans_mat = self.make_transform_matrix("base_link", "l_wrist_roll_link")

            # defbug test
            self.pr2_left_reference_trans_mat = np.eye(4)
            self.pr2_left_reference_trans_mat[:3,3] = np.array([0, 0.3, 1.0])

            pr2_right_transform = self.tf_buffer.lookup_transform("base_link", "r_wrist_roll_link", rospy.Time())
            pr2_right_translation = pr2_right_transform.transform.translation
            self.pr2_right_reference_position = (pr2_right_translation.x, pr2_right_translation.y, pr2_right_translation.z)
            self.pr2_right_reference_trans_mat = self.make_transform_matrix("base_link", "r_wrist_roll_link")

            # debug test
            self.pr2_right_reference_trans_mat = np.eye(4)
            self.pr2_right_reference_trans_mat[:3,3] = np.array([0, -0.3, 1.0])

            rospy.loginfo("\033[32mSet reference position\033[0m")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF transform lookup failed in set reference")


    def update_relative_position(self):
        try:
            # VR tf
            vive_left_transform = self.tf_buffer.lookup_transform("world", "controller_LHR_9F5D6499", rospy.Time(0))
            vive_left_translation = vive_left_transform.transform.translation
            vive_left_rotation = vive_left_transform.transform.rotation
            self.vive_left_relative_position = (
                vive_left_translation.x - self.vive_left_reference_position[0], 
                vive_left_translation.y - self.vive_left_reference_position[1], 
                vive_left_translation.z - self.vive_left_reference_position[2])
            self.vive_left_now_trans_mat = self.make_transform_matrix("world", "controller_LHR_9F5D6499")
            self.set_tf_broadcast("world","vive_left_relative_tf", self.vive_left_relative_position, vive_left_rotation)

            vive_right_transform = self.tf_buffer.lookup_transform("world", "controller_LHR_5920DD6C", rospy.Time(0))
            vive_right_translation = vive_right_transform.transform.translation
            vive_right_rotation = vive_right_transform.transform.rotation
            self.vive_right_relative_position = (
                vive_right_translation.x - self.vive_right_reference_position[0], 
                vive_right_translation.y - self.vive_right_reference_position[1], 
                vive_right_translation.z - self.vive_right_reference_position[2])
            self.vive_right_now_trans_mat = self.make_transform_matrix("world", "controller_LHR_5920DD6C")
            self.set_tf_broadcast("world", "vive_right_relative_tf", self.vive_right_relative_position, vive_right_rotation)

            # make translate matrix
            self.pr2_left_now_trans_mat = self.multiple_matrix(self.pr2_left_reference_trans_mat, self.vive_left_reference_trans_mat, self.vive_left_now_trans_mat)
            self.pr2_right_now_trans_mat = self.multiple_matrix(self.pr2_right_reference_trans_mat, self.vive_right_reference_trans_mat, self.vive_right_now_trans_mat)
            translate_mat_left, rotation_mat_left = self.matrix_to_translation_quaternion(self.pr2_left_now_trans_mat)
            translate_mat_right, rotation_mat_right = self.matrix_to_translation_quaternion(self.pr2_right_now_trans_mat)
            self.set_tf_broadcast("base_link", "pr2_left_matrix_tf", translate_mat_left, rotation_mat_left)
            self.set_tf_broadcast("base_link", "pr2_right_matrix_tf", translate_mat_right, rotation_mat_right)

            # option: 90 degree rotation version
            quaternion = tft.quaternion_from_euler(math.pi/2.0, math.pi/2.0, math.pi)
            broadcaster = tf.TransformBroadcaster()
            broadcaster.sendTransform((0,0,0),quaternion, rospy.Time.now(), "pr2_left_matrix_tf_rotate", "pr2_left_matrix_tf")
            broadcaster.sendTransform((0,0,0),quaternion, rospy.Time.now(), "pr2_right_matrix_tf_rotate", "pr2_right_matrix_tf")

            # vive to pr2 convert
            if self.pr2_left_reference_position is not None and self.pr2_right_reference_position is not None:
                vive_left_relative_position_homogenious_pos = np.array([self.vive_left_relative_position + (1,)])
                vive_right_relative_position_homogenious_pos = np.array([self.vive_right_relative_position + (1,)])

                self.pr2_left_relative_position = np.dot(self.pr2_left_now_trans_mat, vive_left_relative_position_homogenious_pos.reshape(4))
                self.pr2_right_relative_position = np.dot(self.pr2_right_now_trans_mat, vive_right_relative_position_homogenious_pos.reshape(4))
                pr2_left_transform = self.tf_buffer.lookup_transform("base_link", "l_wrist_roll_link", rospy.Time())
                pr2_left_rotation = pr2_left_transform.transform.rotation
                pr2_right_transform = self.tf_buffer.lookup_transform("base_link", "r_wrist_roll_link", rospy.Time())
                pr2_right_rotation = pr2_right_transform.transform.rotation

                self.pr2_left_target_position = (
                    self.pr2_left_reference_position[0] + self.pr2_left_relative_position[0],
                    self.pr2_left_reference_position[1] + self.pr2_left_relative_position[1],
                    self.pr2_left_reference_position[2] + self.pr2_left_relative_position[2])
                self.pr2_right_target_position = (
                    self.pr2_right_reference_position[0] + self.pr2_right_relative_position[0],
                    self.pr2_right_reference_position[1] + self.pr2_right_relative_position[1],
                    self.pr2_right_reference_position[2] + self.pr2_right_relative_position[2])
                
                self.publish(list(self.pr2_left_target_position), self.pr2_left_target_pub)
                self.publish(list(self.pr2_right_target_position), self.pr2_right_target_pub)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed in real-time")
            self.pr2_left_target_position = None
            self.pr2_right_target_position = None

#----------------------------------------------------------------------------------------------------
    def make_transform_matrix(self, target_frame, source_frame):
        # make transform matrix from translate and rotation
        mat = np.eye(4)
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
            R = np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
                        [2*qx*qy + 2*qw*qz, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qw*qx],
                        [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx*qx - 2*qy*qy]])
            mat[:3,:3] = R
            mat[:3,3] = [translation.x, translation.y, translation.z]
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Fail to make transform matrix")
        return mat
    

    def multiple_matrix(self, pr2_reference_mat, vive_reference_mat, vive_now_mat):
        # make robot realtime translation matrix
        inverse_vive_reference_mat = np.linalg.inv(vive_reference_mat)
        pr2_now_trans_mat = np.dot(np.dot(pr2_reference_mat, inverse_vive_reference_mat), vive_now_mat)
        return pr2_now_trans_mat
    
    def matrix_to_translation_quaternion(self, matrix):
        translation_vector = matrix[:3,3]
        rotation_matrix = matrix[:3,:3]
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        return translation_vector, quaternion

#---------------------------------------------------------------------------------------
    def set_tf_broadcast(self, base_frame, child_frame_name, translation, rotation): 
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = base_frame
        transform.child_frame_id = child_frame_name
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        if hasattr(rotation, "x"):
            transform.transform.rotation = rotation
        else:
            transform.transform.rotation.x = rotation[0]
            transform.transform.rotation.y = rotation[1]
            transform.transform.rotation.z = rotation[2]
            transform.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(transform)
  

    def publish(self, data, publisher):
        msg = Float32MultiArray()
        msg.data = data
        publisher.publish(msg)

if __name__ == "__main__":
    try:
        vive_rosbag_recorder = ViveToPR2Converter()
        vive_rosbag_recorder.send_cmd()
    except rospy.ROSInitException:
        pass