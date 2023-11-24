#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Joy
from std_msgs.msg import String

"""
vive reading relative position
"""

class ViveTrack:
    def __init__(self):
        rospy.init_node("vive_send_cmd")
        rospy.loginfo("vive_send_cmd_node start")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.reset()

        self.right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.Joycallback)

    def reset(self):
        self.is_robotswhich = False
        self.right_button = None
        self.robot_cmd = None
        self.relative_left_hand_position = None
        self.relative_right_hand_position = None
        self.reference_left_hand_position = None
        self.reference_right_hand_position = None

    def Joycallback(self, msg):
        self.right_button = msg.buttons[0]

    def send_cmd(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.right_button == 0:
                self.robot_cmd = "OFF"
                if self.is_robotswhich == False:
                    pass
                elif self.is_robotswhich == True:
                    self.is_robotswhich = False

            elif self.right_button == 1:
                self.robot_cmd = "ON"
                if self.is_robotswhich == False:
                    self.is_robotswhich = True
                    self.set_reference_position()
                elif self.is_robotswhich == True:
                    pass
            
            if self.is_robotswhich == True:
                self.update_relative_hand_position()

            if self.relative_left_hand_position is not None:
                print("Relative Left Hand Position:", self.relative_left_hand_position)
            if self.relative_right_hand_position is not None:
                print("Relative Right Hand Position:", self.relative_right_hand_position)

            rate.sleep()
    

    def set_reference_position(self):
        try:
            right_hand_transform = self.tf_buffer.lookup_transform("world", "controller_LHR_5920DD6C", rospy.Time())
            right_hand_translation = right_hand_transform.transform.translation
            self.reference_right_hand_position = (right_hand_translation.x, right_hand_translation.y, right_hand_translation.z)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF transform lookup failed")


    def update_relative_hand_position(self):
        try:
            right_hand_transform = self.tf_buffer.lookup_transform("world", "controller_LHR_5920DD6C", rospy.Time())
            right_hand_translation = right_hand_transform.transform.translation
            self.relative_right_hand_position = (
                right_hand_translation.x - self.reference_right_hand_position[0], 
                right_hand_translation.y - self.reference_right_hand_position[1], 
                right_hand_translation.z - self.reference_right_hand_position[2])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF transform lookup failed")



if __name__ == "__main__":
    try:
        vive_tracking = ViveTrack()
        vive_tracking.send_cmd()
    except rospy.ROSInitException:
        pass