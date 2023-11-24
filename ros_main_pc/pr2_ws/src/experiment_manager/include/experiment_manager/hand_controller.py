#! /usr/bin/env python3

import rospy
from pr2_controllers_msgs.msg import Pr2GripperCommand

class PR2GripperController:
    def __init__(self):
        self.open_pos = 0.09
        self.close_pos = 0.00
        self.max_effort = 50.0

        self.left_gripper_pub = rospy.Publisher("/l_gripper_controller/command", Pr2GripperCommand, queue_size=10)
        self.right_gripper_pub = rospy.Publisher("/r_gripper_controller/command", Pr2GripperCommand, queue_size=10)

    def make_goal(self, pos):
        gripper_cmd = Pr2GripperCommand()
        gripper_cmd.position = pos
        gripper_cmd.max_effort = self.max_effort
        return gripper_cmd

    def left_open(self):
        gripper_cmd = self.make_goal(self.open_pos)
        self.left_gripper_pub.publish(gripper_cmd)

    def right_open(self):
        gripper_cmd = self.make_goal(self.open_pos)
        self.right_gripper_pub.publish(gripper_cmd)

    def left_close(self):
        gripper_cmd = self.make_goal(self.close_pos)        
        self.left_gripper_pub.publish(gripper_cmd)
        
    def right_close(self):
        gripper_cmd = self.make_goal(self.close_pos)        
        self.right_gripper_pub.publish(gripper_cmd)

def main():
    rospy.init_node("hand_controller_node")
    rospy.loginfo("hand_controller_node start")

    hand_controller = PR2GripperController()

    while not rospy.is_shutdown():
        rospy.sleep(1)
        hand_controller.left_open()
        hand_controller.right_open()

        rospy.sleep(1)
        hand_controller.left_close()
        hand_controller.right_close()

        rospy.sleep(3)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass