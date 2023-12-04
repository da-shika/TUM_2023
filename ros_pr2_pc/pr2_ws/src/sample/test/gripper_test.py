#! /usr/bin/env python

import time
import rospy
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

class PR2GripperController:
    def __init__(self):
        rospy.init_node("pr2_gripper_controller")
        rospy.loginfo("pr2_gripper_controller_node start")

        self.client = actionlib.SimpleActionClient("/l_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.client.wait_for_server()

    def open(self, max_effort=50.0):
        gripper_cmd = Pr2GripperCommandGoal()
        gripper_cmd.command.position = 0.09
        gripper_cmd.command.max_effort = max_effort
        
        self.client.send_goal(gripper_cmd)
        self.client.wait_for_result()

    def close(self, max_effort=50.0):
        gripper_cmd = Pr2GripperCommandGoal()
        gripper_cmd.command.position = 0.00
        gripper_cmd.command.max_effort = max_effort
        
        self.client.send_goal(gripper_cmd)
        self.client.wait_for_result()

def main():
    pr2_gripper = PR2GripperController()
    rospy.sleep(1)

    s = time.time()
    pr2_gripper.open()
    print(time.time() - s)
    rospy.sleep(5)
    pr2_gripper.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass