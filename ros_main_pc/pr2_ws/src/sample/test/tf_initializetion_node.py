#! /usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
Initializetion of VR hand
"""

class VRInitialize:
    def __init__(self, base_frame, device_frame):
        rospy.init_node("VR_initialiye_node", anonymous=True)
        rospy.loginfo("VR_initialiye_node start")

        self.base_frame = base_frame
        self.device_frame = device_frame
        
        self.reset()
        self.listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

    def reset(self):
        self.position_offset = [0.0, 0.0, 0.0]
        self.rotation_offset = [0.0, 0.0, 0.0]

    def set_position_offset(self, position_offset):
        self.position_offset = position_offset

    def set_rotation_offset(self, rotation_offset):
        self.rotation_offset = rotation_offset

    def perform_initial_alignment(self):
        try:
            self.listener.waitForTransform(self.base_frame, self.device_frame, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform(self.base_frame, self.device_frame, rospy.Time(0))

            trans[0] += self.position_offset[0]
            trans[1] += self.position_offset[1]
            trans[2] += self.position_offset[2]

            euler = euler_from_quaternion(rot)
            euler = [a + b for a, b in zip(euler, self.rotation_offset)]
            rot = quaternion_from_euler(euler[0], euler[1], euler[2])

            # broadcast new tf
            self.tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), self.device_frame+"_ad", self.base_frame)

            rospy.loginfo("Initialize complete")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error during initialization: {}", format(str(e)))
        
if __name__ == "__main__":
    base_frame = "/world"
    left_device_frame = "/controller_LHR_9F5D6499"
    right_device_name = "/controller_LHR_5920DD6C"

    """left_initialize = VRInitialize(base_frame, left_device_frame)
    left_initialize.set_position_offset([0.5, 0.0, 0.0])
    left_initialize.set_rotation_offset([0.0, 0.0, 0.785])"""

    right_initialize = VRInitialize(base_frame, right_device_name)
    right_initialize.set_position_offset([0.5, 0.0, 0.0])
    right_initialize.set_rotation_offset([0.0, 0.0, 0.785])
    right_initialize.perform_initial_alignment()

    rospy.spin()