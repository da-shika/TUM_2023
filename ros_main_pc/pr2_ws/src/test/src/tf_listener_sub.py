#! /usr/bin/env python3

"""
get vr offset of both VR hand
"""

import math
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

class VRTFListener:
    def __init__(self):
        rospy.init_node("VR_offset_listener")
        rospy.loginfo("VR_offset_listener_node start")
        
        self.vive_left_current_pos = None
        self.vive_right_current_pos = None
        self.vive_left_current_rot = None
        self.vive_right_current_rot = None
        self.right_button = None

        self.left_vive_position_sub = rospy.Subscriber("/vive/poseLeft", PoseStamped, self.left_vive_callback)
        self.right_vive_position_sub = rospy.Subscriber("/vive/poseRight", PoseStamped, self.right_vive_callback)
        self.right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.Joy_callback)


    def left_vive_callback(self, msg):
        pos = msg.pose.position
        rot = msg.pose.orientation
        self.vive_left_current_pos = [pos.x, pos.y, pos.z]
        self.vive_left_current_rot = [rot.x, rot.y, rot.z, rot.w]
    def right_vive_callback(self, msg):
        pos = msg.pose.position
        rot = msg.pose.orientation
        self.vive_right_current_pos = [pos.x, pos.y, pos.z]
        self.vive_right_current_rot = [rot.x, rot.y, rot.z, rot.w]
    def Joy_callback(self, msg):
        self.right_button = msg.buttons[0]

    
    def run(self):
        if self.right_button == 1:
            rospy.loginfo(f"Translation:Left, {self.vive_left_current_pos}")
            rospy.loginfo(f"Rotatation:Left, {self.vive_left_current_rot}")
            rospy.loginfo(f"Translation:Right, {self.vive_right_current_pos}")
            rospy.loginfo(f"Rotatation:Right, {self.vive_right_current_rot}")


if __name__ == "__main__":
    tf_listener = VRTFListener()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        tf_listener.run()
        rate.sleep()