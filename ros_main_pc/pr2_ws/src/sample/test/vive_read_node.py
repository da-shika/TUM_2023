#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
read vr buttons
"""

import rospy
from sensor_msgs.msg import Joy

def main():
    rospy.init_node("vive_controller_sub_node")
    rospy.loginfo("vive_controller_sub_node start")

    left_vive_sub = rospy.Subscriber("/vive/controller_LHR_9F5D6499/joy", Joy, Joycallbackleft)
    right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, Joycallbackright)

    rospy.spin()

def Joycallbackleft(msg):
    b = msg.buttons
    print(b, "left")

def Joycallbackright(msg):
    b = msg.buttons
    print(b, "right")

if __name__ ==  "__main__":
    main()