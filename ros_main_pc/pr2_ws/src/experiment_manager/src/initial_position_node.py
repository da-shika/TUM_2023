#! /usr/bin/env python3

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class InitialPosition:
    def __init__(self):
        rospy.init_node("initial_position_node")
        rospy.loginfo("initial_position_node start")

        self.left_initial_position = np.array([-0.0045037736244504245, -0.00011895716422749558, 0.006574954580981007, 7.165849240564626e-05,
                                               -0.306538116205914, -0.43759435803385127, 0.0003388245909397014])
        self.right_initial_position = np.array([0.0015158762067342124, -3.3892049154715664e-05, 0.0066137725178680995, 0.00024150218120677636,
                                                -0.3075269110404957, -0.4364114258855327, -0.00020473830900602508])