#! /usr/bin/env python3

import math
import numpy as np
import subprocess
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import  PoseStamped

PI = math.pi

class VivePR2Controller:
    def __init__(self):
        rospy.init_node("vive_pr2_position_controller")
        rospy.loginfo("vive_pr2_position_controller start")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.config()
        self.config_position()
        self.config_marix()

        self.record_loop = 1

        self.left_vive_button_sub = rospy.Subscriber("/vive/controller_LHR_9F5D6499/joy", Joy, self.left_button_callback)
        self.right_vive_button_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.right_button_callback)
        self.left_vive_position_sub = rospy.Subscriber("/vive/poseLeft", PoseStamped, self.left_vive_pos_callback)
        self.right_vive_position_sub = rospy.Subscriber("/vive/poseRight", PoseStamped, self.right_vive_pos_callback)

        self.IO_cmd_pub = rospy.Publisher("/robot_IO_cmd", String, queue_size=10)
        self.initial_pos_cmd_pub = rospy.Publisher("/robot_initial_pos_cmd", String, queue_size=10)
        self.left_gripper_cmd_pub = rospy.Publisher("pr2_left_gripper_cmd", Float32, queue_size=10)
        self.right_gripper_cmd_pub = rospy.Publisher("pr2_right_gripper_cmd", Float32, queue_size=10)
        self.pr2_left_target_pos_pub = rospy.Publisher("/pr2_left_target_pos", Float32MultiArray, queue_size=10)
        self.pr2_right_target_pos_pub = rospy.Publisher("/pr2_right_target_pos", Float32MultiArray, queue_size=10)

    def config(self):
        self.is_robot_on = False
        self.is_recording = False
        self.is_robot_initial = True
        self.is_left_hand_open = True
        self.is_right_hand_open = True
        self.is_first_loop = True

        self.robot_IO_cmd = None
        self.recording_cmd = None
        self.robot_initial_pos_cmd = None
        self.left_hand_cmd = None
        self.right_hand_cmd = None

        self.right_IO_button = None             # r_vive[0]
        self.left_IO_button = None              # l_vive[0]
        self.right_record_button = None         # r_vive[2]
        self.left_record_button = None          # l_vive[2]
        self.right_initial_pos_button = None    # r_vive[3]
        self.left_gripper_button = None         # l_vive[1]
        self.right_gripper_button = None        # r_vive[1]
    
    def config_position(self):
        self.vive_left_current_pos = None
        self.vive_right_current_pos = None

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

    def config_marix(self):
        """
        caliculate Rotation of wrist to vive
        R_wrist_to_vive = (R_vive_to_world)^-1 * R_base_to_world * R_wrist_to_base
        """
        def rot_mat_to_quaternion(rot_mat):
            return Rotation.from_matrix(rot_mat).as_quat()
        def quaternion_to_conjugate(q):
            return np.array([-q[0], -q[1], -q[2], q[3]])

        self.R_world_to_base = np.array([[0,-1,0],
                                        [1,0,0],
                                        [0,0,1]])
        self.q_world_to_base = rot_mat_to_quaternion(self.R_world_to_base)
        q_base_to_world = quaternion_to_conjugate(self.q_world_to_base)

        q_left_wrist_to_base = np.array([-0.0014864, -0.35162, 0.00078016, 0.93614])
        q_left_vive_to_world = np.array([0.04503167541099551, 0.9717776070935632, 0.23153466555570307, 0.0034827011349717156])  #same direction with robot
        q_right_wrist_to_base = np.array([0.00057509, -0.35149, -0.00039639, 0.93619])
        q_right_vive_to_world = np.array([0.09690744189581683, 0.9550075977364274, 0.27434072359528616, 0.05750307251027345])    #same direction with robot

        q_left_vive_to_world_conjugate = quaternion_to_conjugate(q_left_vive_to_world)
        q_right_vive_to_world_conjugate = quaternion_to_conjugate(q_right_vive_to_world)

        self.q_left_wrist_to_vive = self.quaternion_multiply(self.quaternion_multiply(q_left_vive_to_world_conjugate, q_base_to_world), q_left_wrist_to_base)
        self.q_right_wrist_to_vive = self.quaternion_multiply(self.quaternion_multiply(q_right_vive_to_world_conjugate, q_base_to_world), q_right_wrist_to_base)


    def left_vive_pos_callback(self, msg):
        pos = msg.pose.position
        self.vive_left_current_pos = [pos.x, pos.y, pos.z]
    def right_vive_pos_callback(self, msg):
        pos = msg.pose.position
        self.vive_right_current_pos = [pos.x, pos.y, pos.z]

    def left_button_callback(self, msg):
        self.left_IO_button = msg.buttons[0]
        self.left_gripper_button = msg.buttons[1]
        self.left_record_button = msg.buttons[2]

    def right_button_callback(self, msg):
        self.right_IO_button = msg.buttons[0]
        self.right_gripper_button = msg.buttons[1]
        self.right_record_button = msg.buttons[2]
        self.right_initial_pos_button = msg.buttons[3]

#-------------------------------------------------------------------------------------------------------------------
    def send_cmd (self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.send_robot_cmd()
            self.send_record_cmd()
            self.send_initial_pos_cmd()
            self.send_hand_cmd()
            rate.sleep()


    def send_robot_cmd(self):
        """
        self.is_robot_on = state of the robot           (bool)
        self.robot_IO_cmd = command of the I/O robot    (ON/OFF)
        """
        if self.is_robot_on == False:
            if self.right_IO_button == 0:
                pass
            elif self.right_IO_button == 1:
                rospy.loginfo("robot switch on")
                self.is_robot_on = True
                self.robot_IO_cmd = "ON"
                self.IO_cmd_pub.publish(self.robot_IO_cmd)
                self.set_reference()
        
        elif self.is_robot_on == True:
            if self.left_IO_button == 0:
                if (self.vive_left_reference_pos is not None and self.vive_right_reference_pos is not None\
                    and self.pr2_right_reference_pos is not None and self.pr2_right_reference_pos is not None):
                    self.update_relative_position()
            elif self.left_IO_button == 1:
                rospy.loginfo("robot switch off")
                self.is_robot_on = False
                self.robot_IO_cmd = "OFF"
                self.IO_cmd_pub.publish(self.robot_IO_cmd)
    
    def send_record_cmd(self):
        rosbag_start_cmd = "rosbag record -O eipl_tutorial_"+str(self.record_loop)+".bag /joint_states /pr2_left_gripper_cmd /pr2_right_gripper_cmd /wide_stereo/left/image_raw/compressed"
        rosbag_stop_cmd = "pkill -f 'rosbag record'"

        if self.is_recording == False:
            if self.right_record_button == 0:
                pass
            elif self.right_record_button == 1:
                rospy.loginfo("recording start")
                self.is_recording = True
                rosbag_process = subprocess.Popen(rosbag_start_cmd, shell=True)
                self.record_loop += 1

        elif self.is_recording == True:
            if self.left_record_button == 0:
                pass
            elif self.left_record_button == 1:
                rospy.loginfo("recording stop")
                self.is_recording = False
                subprocess.call(rosbag_stop_cmd, shell=True)        

    def send_initial_pos_cmd(self):
        if self.is_robot_on == True:
            self.is_robot_initial = False
        elif self.is_robot_on == False:
            if self.is_robot_initial == True:
                pass
            elif self.is_robot_initial == False:
                if self.right_initial_pos_button == 1:
                    rospy.loginfo("Move to initial position")
                    self.robot_initial_pos_cmd = "Move to initial pos"
                    self.initial_pos_cmd_pub.publish(self.robot_initial_pos_cmd)
                    self.is_robot_initial = True
                    rospy.sleep(5)
                    rospy.loginfo("Reach to initial position")

    def send_hand_cmd(self):
        def pub_float32(data, publisher):
            msg = Float32()
            msg.data = data
            publisher.publish(msg)
        pub_float32(self.left_gripper_button, self.left_gripper_cmd_pub)
        pub_float32(self.right_gripper_button, self.right_gripper_cmd_pub)

#-------------------------------------------------------------------------------------------------------------------
    def set_reference(self):
        # get initial position and rotation
        self.vive_left_reference_pos = self.vive_left_current_pos
        self.vive_right_reference_pos = self.vive_right_current_pos
        self.pr2_left_reference_pos, _ = self.get_trans_rot_from_tf("base_link", "l_wrist_roll_link")
        self.pr2_right_reference_pos, _ = self.get_trans_rot_from_tf("base_link", "r_wrist_roll_link")
        rospy.loginfo("\033[32mSuccess to set reference position!\033[0m")


    def update_relative_position(self):
        # get relative position of vive
        def get_vive_relative_pos(current, reference):
            relative_0 = current[0] - reference[0]
            relative_1 = current[1] - reference[1]
            relative_2 = current[2] - reference[2]
            return [relative_0, relative_1, relative_2]
        
        self.vive_left_relative_pos = get_vive_relative_pos(self.vive_left_current_pos, self.vive_left_reference_pos)
        self.vive_right_relative_pos = get_vive_relative_pos(self.vive_right_current_pos, self.vive_right_reference_pos)

        # get relative position of pr2
        _pr2_left_relative_pos = np.dot(self.R_world_to_base, self.vive_left_relative_pos)
        _pr2_right_relative_pos = np.dot(self.R_world_to_base, self.vive_right_relative_pos)

        # Add the rotation matrix because I have to trade x and z axis
        R_add = np.array([[0,0,1],
                        [0,1,0],
                        [-1,0,0]])
        self.pr2_left_relative_pos = np.dot(R_add, _pr2_left_relative_pos)
        self.pr2_right_relative_pos = np.dot(R_add, _pr2_right_relative_pos)

        # get pr2 target positions
        def get_pr2_target_pos(reference, relative):
            target_0 = reference[0] + relative[0]
            target_1 = reference[1] + relative[1]
            target_2 = reference[2] + relative[2]
            return [target_0, target_1, target_2]
        
        self.pr2_left_target_pos = get_pr2_target_pos(self.pr2_left_reference_pos, self.pr2_left_relative_pos)
        self.pr2_right_target_pos = get_pr2_target_pos(self.pr2_right_reference_pos, self.pr2_right_relative_pos)

        # publish target pos
        def publish(data, publisher):
            msg = Float32MultiArray()
            msg.data = data
            publisher.publish(msg)

        publish(list(self.pr2_left_target_pos), self.pr2_left_target_pos_pub)
        publish(list(self.pr2_right_target_pos), self.pr2_right_target_pos_pub)

#-------------------------------------------------------------------------------------------------------------------
    def get_trans_rot_from_tf(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            pos = [translation.x, translation.y, translation.z]
            rot = [rotation.x, rotation.y, rotation.z, rotation.w]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF transform lookup failed in set reference")
        return pos, rot

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