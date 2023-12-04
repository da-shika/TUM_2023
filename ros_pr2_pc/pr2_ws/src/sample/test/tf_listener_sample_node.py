#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped

class PR2HandOffsets:
    def __init__(self):
        rospy.init_node("PR2_hand_offsets")
        rospy.loginfo("PR2_hand_offsets_node start")
        self.tf_listener = tf.TransformListener()
        self.left_hand_offset = None
        self.right_hand_offset = None

    def get_hand_offsets(self):
        base_frame = "/base_link"
        left_hand_frame = "/l_wrist_roll_link"
        right_hand_frame = "/r_wrist_roll_link"

        try:
            self.tf_listener.waitForTransform(base_frame, left_hand_frame, rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(base_frame, left_hand_frame, rospy.Time(0))
            self.left_hand_offset = (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logger("Failed to get left hand offset")

        try:
            self.tf_listener.waitForTransform(base_frame, right_hand_frame, rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(base_frame, right_hand_frame, rospy.Time(0))
            self.right_hand_offset = (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logger("Failed to get left hand offset")


    def print_offsets(self):
        if self.left_hand_offset:
            trans, rot = self.left_hand_offset
            print("Left Hand Offset :")
            print("Position Offset:", trans)
            print("Rotation Offset:", rot)
        if self.right_hand_offset:
            trans, rot = self.right_hand_offset
            print("Right Hand Offset :")
            print("Position Offset:", trans)
            print("Rotation Offset:", rot)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.get_hand_offsets()
            self.print_offsets()
            rate.sleep()

if __name__ == "__main__":
    pr2_hand_offsets = PR2HandOffsets()
    pr2_hand_offsets.run()