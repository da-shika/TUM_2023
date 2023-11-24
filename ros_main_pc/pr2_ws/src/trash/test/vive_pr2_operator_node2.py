#! /usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import TransformStamped

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
        self.vive_left_reference_rot = None
        self.vive_right_reference_rot = None
        self.pr2_left_reference_rot = None
        self.pr2_right_reference_rot = None

        self.vive_left_relative_rot = None
        self.vive_right_relative_rot = None
        self.pr2_left_relative_rot = None
        self.pr2_right_relative_rot = None

        self.pr2_left_target_rot = None
        self.pr2_right_target_rot = None

    def config_marix(self):
        self.R = np.array([[0,-1,0],
                           [1,0,0],
                           [0,0,1]])
        rotation = Rotation.from_matrix(self.R)
        self.quaternion_R = rotation.as_quat()

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
        self.vive_left_reference_pos, self.vive_left_reference_rot = self.get_trans_rot_from_tf("world", "controller_LHR_9F5D6499")
        self.vive_right_reference_pos, self.vive_right_reference_rot = self.get_trans_rot_from_tf("world", "controller_LHR_5920DD6C")
        self.pr2_left_reference_pos, self.pr2_left_reference_rot = self.get_trans_rot_from_tf("base_link", "l_wrist_roll_link")
        self.pr2_right_reference_pos, self.pr2_right_reference_rot = self.get_trans_rot_from_tf("base_link", "r_wrist_roll_link")

        rospy.loginfo("\033[32mSuccess to set reference position!\033[0m")


    def update_relative_position(self):
        # get relative position and rotation of vive
        def get_vive_relative_pos(current, reference):
            relative_0 = current[0] - reference[0]
            relative_1 = current[1] - reference[1]
            relative_2 = current[2] - reference[2]
            return (relative_0, relative_1, relative_2)
        
        def get_vive_relative_rot(current, reference):
            def get_reference_inverse(reference):
                q_conjugate = np.array([-reference[0], -reference[1], -reference[2], reference[3]])
                norm_suqared = np.dot(reference, reference)
                q_inverse = q_conjugate / norm_suqared
                return q_inverse        
            reference_inverse = get_reference_inverse(reference)
            relative = self.quaternion_multiply(np.array(current), reference_inverse)
            relative /= np.linalg.norm(relative)
            return relative


        vive_left_current_translation, vive_left_current_rotation = self.get_trans_rot_from_tf("world", "controller_LHR_9F5D6499")
        self.vive_left_relative_pos = get_vive_relative_pos(vive_left_current_translation, self.vive_left_reference_pos)
        self.vive_left_relative_rot = get_vive_relative_rot(vive_left_current_rotation, self.vive_left_reference_rot)

        vive_right_current_translation, vive_right_current_rotation = self.get_trans_rot_from_tf("world", "controller_LHR_5920DD6C")
        self.vive_right_relative_pos = get_vive_relative_pos(vive_right_current_translation, self.vive_right_reference_pos)
        self.vive_right_relative_rot = get_vive_relative_rot(vive_right_current_rotation, self.vive_right_reference_rot)
        
        # get relative position and rotation of pr2
        self.pr2_left_relative_pos = np.dot(self.R, self.vive_left_relative_pos)
        self.pr2_right_relative_pos = np.dot(self.R, self.vive_right_relative_pos)

        def translate_vive_relative_quaternion_to_pr2(vive_relative):
            rot = Rotation.from_quat(vive_relative)
            rot_matrix = rot.as_matrix()
            transformed_matrix = np.dot(self.R, rot_matrix)
            transformed_rot = Rotation.from_matrix(transformed_matrix)
            transformed_q = transformed_rot.as_quat()
            return transformed_q

        self.pr2_left_relative_rot = translate_vive_relative_quaternion_to_pr2(self.vive_left_relative_rot)
        self.pr2_right_relative_rot = translate_vive_relative_quaternion_to_pr2(self.vive_right_relative_rot)

        # visualize pr2 relative
        self.set_tf_broadcast("base_link", "pr2_left_relative_tf", self.pr2_left_relative_pos, self.pr2_left_relative_rot)
        self.set_tf_broadcast("base_link", "pr2_right_relative_tf", self.pr2_right_relative_pos, self.pr2_right_relative_rot)

        # get pr2 target positions
        def get_pr2_target_pos(reference, relative):
            target_0 = reference[0] + relative[0]
            target_1 = reference[1] + relative[1]
            target_2 = reference[2] + relative[2]
            return (target_0, target_1, target_2)
        
        def get_pr2_target_rot(reference, relative):
            target_rot = self.quaternion_multiply(relative, reference)
            target_rot /= np.linalg.norm(target_rot)
            return target_rot
        
        self.pr2_left_target_pos = get_pr2_target_pos(self.pr2_left_reference_pos, self.pr2_left_relative_pos)
        self.pr2_right_target_pos = get_pr2_target_pos(self.pr2_right_reference_pos, self.pr2_right_relative_pos)
        self.pr2_left_target_rot = get_pr2_target_rot(self.pr2_left_reference_rot, self.pr2_left_relative_rot)
        self.pr2_right_target_rot = get_pr2_target_rot(self.pr2_right_reference_rot, self.pr2_right_relative_rot)

        # visualize pr2 target
        self.set_tf_broadcast("base_link", "pr2_left_target_tf", self.pr2_left_target_pos, self.pr2_left_target_rot)
        self.set_tf_broadcast("base_link", "pr2_right_target_tf", self.pr2_right_target_pos, self.pr2_right_target_rot)

        # publish target pos
        def publish(data, publisher):
            msg = Float32MultiArray()
            msg.data = data
            publisher.publish(msg)

        publish(list(self.pr2_left_target_pos), self.pr2_left_target_pos_pub)
        publish(list(self.pr2_right_target_pos), self.pr2_right_target_pos_pub)
        publish(list(self.pr2_left_target_rot), self.pr2_left_target_rot_pub)
        publish(list(self.pr2_right_target_rot), self.pr2_right_target_rot_pub)

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