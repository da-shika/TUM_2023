#! /usr/bin/env python3

"""
get vr offset of both VR hand
"""

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion

class VRTFOffsetListener:
    def __init__(self, base_frame, device_frame, side):
        self.base_frame = base_frame
        self.device_frame = device_frame
        self.side = side
        self.listener = tf.TransformListener()
        
        self.offset_sub = rospy.Subscriber("/tf", TransformStamped, self.offsetcallback)

    def offsetcallback(self, tf):
        try:
            (trans, rot) = self.listener.lookupTransform(self.base_frame, self.device_frame, rospy.Time(0))

            tf_msg = TransformStamped()
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = self.base_frame
            tf_msg.child_frame_id = self.device_frame
            
            tf_msg.transform.translation.x = trans[0]
            tf_msg.transform.translation.y = trans[1]
            tf_msg.transform.translation.z = trans[2]
            tf_msg.transform.rotation.x = rot[0]
            tf_msg.transform.rotation.y = rot[1]
            tf_msg.transform.rotation.z = rot[2]
            tf_msg.transform.rotation.w = rot[3]

            # get position offset
            position_offset = (trans[0], trans[1], trans[2])

            # get rotation offset (quaternion to euler)
            rotation_quaternion = (rot[0], rot[1], rot[2], rot[3])
            euler_angles = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
            rotation_offset = (euler_angles[0], euler_angles[1], euler_angles[2])

            rospy.loginfo(f"Rotation Quaternion:, {self.side}:{rotation_quaternion}")
            """rospy.loginfo(f"Position Offset: {self.side}: {position_offset}")
            rospy.loginfo(f"Rotation Offset: {self.side}:{rotation_offset}")"""

        except (tf.LookupException, tf.ConnectiveException, tf.ExtrapolationException):
            rospy.logwarn("Transform lookup failed")

if __name__ == "__main__":
    rospy.init_node("VR_offset_listener")
    rospy.loginfo("VR_offset_listener_node start")

    base_frame = "/world"
    left_device_frame = "/controller_LHR_9F5D6499"
    right_device_frame = "/controller_LHR_5920DD6C"
    left_vr_listener = VRTFOffsetListener(base_frame, left_device_frame, "left")
    #right_vr_listener = VRTFOffsetListener(base_frame, right_device_frame, "right")

    rospy.spin()