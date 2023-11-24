#! /usr/bin/env python3

"""
controll rosbag cmd by VR button
"""

import rospy
import subprocess
from sensor_msgs.msg import Joy

class ViveRosbagRecorder:
    def __init__(self):
        rospy.init_node("vive_rosbag_recorder")
        rospy.loginfo("vive_rosbag_recorder_node start")

        self.reset()
        self.rosbag_start_cmd = "rosbag record /vive/controller_LHR_5920DD6C/joy"
        self.rosbag_stop_cmd = "pkill -f 'rosbag record'"

        right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.Joycallback)

    def reset(self):
        self.is_recording = False
        self.right_button = None

    def Joycallback(self, msg):
        self.right_button = msg.buttons[0]

    def logger(self):
        while not rospy.is_shutdown():
            if self.right_button == 0:
                if self.is_recording == False:
                    pass
                elif self.is_recording == True:
                    rospy.loginfo("stop recording")
                    self.is_recording = False
                    subprocess.call(self.rosbag_stop_cmd, shell=True)

            elif self.right_button == 1:
                if self.is_recording == False:
                    rospy.loginfo("start recording")
                    self.is_recording = True
                    rosbag_process = subprocess.Popen(self.rosbag_start_cmd, shell=True)
                elif self.is_recording == True:
                    pass

if __name__ == "__main__":
    try:
        vive_rosbag_recorder = ViveRosbagRecorder()
        vive_rosbag_recorder.logger()
    except rospy.ROSInitException:
        pass