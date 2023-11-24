#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

"""
test_code to send robot ON/OFF
"""

class ViveSendCmd:
    def __init__(self):
        rospy.init_node("vive_send_cmd")
        rospy.loginfo("vive_send_cmd_node start")

        self.reset()

        self.right_vive_sub = rospy.Subscriber("/vive/controller_LHR_5920DD6C/joy", Joy, self.Joycallback)
        self.send_cmd_pub = rospy.Publisher("/robot_IO_cmd", String, queue_size=10)

    def reset(self):
        self.is_robotswhich = False
        self.right_button = None
        self.robot_cmd = None

    def Joycallback(self, msg):
        self.right_button = msg.buttons[0]

    def send_cmd(self):
        while not rospy.is_shutdown():
            if self.right_button == 0:
                self.robot_cmd = "OFF"
                if self.is_robotswhich == False:
                    pass
                elif self.is_robotswhich == True:
                    rospy.loginfo("robot switch off")
                    self.is_robotswhich = False
                    self.send_cmd_pub.publish(self.robot_cmd)

            elif self.right_button == 1:
                self.robot_cmd = "ON"
                if self.is_robotswhich == False:
                    rospy.loginfo("robot switch on")
                    self.is_robotswhich = True
                    self.send_cmd_pub.publish(self.robot_cmd)
                elif self.is_robotswhich == True:
                    pass

if __name__ == "__main__":
    try:
        vive_rosbag_recorder = ViveSendCmd()
        vive_rosbag_recorder.send_cmd()
    except rospy.ROSInitException:
        pass