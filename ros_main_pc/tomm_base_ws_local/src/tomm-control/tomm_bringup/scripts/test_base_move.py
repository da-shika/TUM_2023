#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np

import behavior_msgs.msg as behavior
import geometry_msgs.msg

import ros_utilities as ros_utils

class BaseMoveClient:
    """Set new motion goal for motion task
    """
    def __init__(self, name, timeout=30):
        self.is_done = False
        self.client = actionlib.SimpleActionClient(name, behavior.BaseMoveAction)
        if not self.client.wait_for_server(rospy.Duration(timeout)):
            raise Exception('Cant reach Action server: {:1}'.format(name))

        self.goal_pose_pub = rospy.Publisher(name + '/goal_pose', geometry_msgs.msg.Vector3, queue_size=1)
        self.goal = None
        self.final_pose = None

    def sendGoal(self, period, pose):
        ''' send a new goal '''
        # clear old feedbacks
        self.start_time = rospy.Time.now()

        # publish the goal pose for visualization
        pose_msg = geometry_msgs.msg.Vector3()
        pose_msg.x = pose[0]
        pose_msg.y = pose[1]
        pose_msg.z = pose[2]
        self.goal_pose_pub.publish(pose_msg)

        # create the goal
        self.goal = behavior.MoveToPoseGoal()
        self.goal.period.data = period
        self.goal.target = pose_msg

        # send the goal
        self.client.send_goal(self.goal,
            feedback_cb=self.__callback_fb,
            done_cb=self.__callback_done)

    def wait(self):
        self.client.wait_for_result()
        return self.result()

    def __callback_fb(self, feedback):
        rospy.loginfo_throttle(0.5, feedback.pose)

    def __callback_done(self, state, result):
        self.is_done = True
        result = behavior.BaseMoveResult()
        self.final_pose = result.pose
        if state != actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('MoveToCartesianClient failed')
        else:
            rospy.loginfo('MoveToCartesianClient success')

    def result(self):
        return self.final_pose 

class TommClient:
    def __init__(self):
        self.base = BaseMoveClient('/tomm/base_move')

    def move(self, period, pose, block=True):
        self.base.sendGoal(period, pose)
        print("base goal sent")
        if block:
            return self.base.wait()

def main():
    rospy.init_node('test_approch_object')
    rospy.sleep(1.0)

    tomm = TommClient()

    #---------------------------------------------------------------------------
    # reach the start pose

    period = 6.0

    pose = np.array([0.1, 0.0, np.pi/9])
    tomm.move(period, pose)

    rospy.spin()

if __name__ == '__main__':
    main()