#! /usr/bin/env python3

"""
get vr offset of both VR hand
"""

import math
import numpy as np
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped

PI = math.pi

class VRTFListener:
    def __init__(self):
        self.config()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.Joycallback)

    def config(self):
        self.right_button = None

    def Joycallback(self, msg):
        self.right_button = msg.buttons[0]

    
    def tf_reader(self, side, target_frame, source_frame):
        def tf_rotater(rotation_degree, q1):
            r = Rotation.from_euler("xyz", rotation_degree, degrees=False)
            q2 = r.as_quat()
            return self.quaternion_multiply(q1, q2)

        if self.right_button == 1:
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                pos = (translation.x, translation.y, translation.z)
                rot = (rotation.x, rotation.y, rotation.z, rotation.w)
                
                new_rot = tf_rotater((PI, PI/2.0, PI), rot)

                rospy.loginfo(f"Translation: {side}:{pos}")
                rospy.loginfo(f"Rotatation: {side}:{rot}")
                self.set_tf_broadcast(target_frame, "pr2_"+side+"_target_tf", pos, new_rot)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("TF transform lookup failed")
        else:
            pass


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


if __name__ == "__main__":
    rospy.init_node("VR_offset_listener")
    rospy.loginfo("VR_offset_listener_node start")
    tf_listener = VRTFListener()
    rate = rospy.Rate(60)

    base_frame = "world"
    left_device_frame = "controller_LHR_9F5D6499"
    right_device_frame = "controller_LHR_5920DD6C"

    while not rospy.is_shutdown():
        left_vr_listener = tf_listener.tf_reader("left", base_frame, left_device_frame)
        right_vr_listener = tf_listener.tf_reader("right", base_frame, right_device_frame)
        rate.sleep()