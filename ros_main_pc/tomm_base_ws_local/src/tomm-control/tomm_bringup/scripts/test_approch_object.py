#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np

import behavior_msgs.msg as behavior
import geometry_msgs.msg

import ros_utilities as ros_utils

class MoveToCartesianClient:
    """Set new motion goal for motion task
    """
    def __init__(self, name, timeout=30):
        self.is_done = False
        self.client = actionlib.SimpleActionClient(name, behavior.MoveToPoseAction)
        if not self.client.wait_for_server(rospy.Duration(timeout)):
            raise Exception('Cant reach Action server: {:1}'.format(name))

        self.goal_pose_pub = rospy.Publisher(name + '/goal_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.goal = None
        self.final_pose = None

    def sendGoal(self, period, pos, orien, frame, task):
        ''' send a new goal '''
        # clear old feedbacks
        self.start_time = rospy.Time.now()

        # publish the goal pose for visualization
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = self.start_time
        pose.header.frame_id = frame
        pose.pose = ros_utils.to_pose_msg(pos, orien)
        self.goal_pose_pub.publish(pose)

        # create the goal
        self.goal = behavior.MoveToPoseGoal()
        self.goal.motion_task.data = task
        self.goal.period.data = period
        self.goal.target.header.stamp = self.start_time
        self.goal.target.header.frame_id = frame
        self.goal.target.pose = pose.pose

        # send the goal
        self.client.send_goal(self.goal,
            feedback_cb=None,
            done_cb=self.__callback_done)

    def wait(self):
        self.client.wait_for_result()
        return self.result()

    def __callback_done(self, state, result):
        self.is_done = True
        result = behavior.MoveToPoseResult()
        pos, orien = ros_utils.pose_to_np(result.pose.pose)
        self.final_pose = np.concatenate([pos, orien])
        if state != actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('MoveToCartesianClient failed')
        else:
            rospy.loginfo('MoveToCartesianClient success')

    def result(self):
        return self.final_pose 

class TommClient:
    def __init__(self):
        self.left_hand = MoveToCartesianClient('/tomm/reach_goal')
        self.right_hand = MoveToCartesianClient('/tomm/reach_goal')

    def moveRightHand(self, period, pos, orien, frame, block=True):
        self.right_hand.sendGoal(period, pos, orien, frame, 'right_hand_motion')
        print("right hand goal sent")
        if block:
            return self.right_hand.wait()

    def moveLeftHand(self, period, pos, orien, frame, block=True):
        self.left_hand.sendGoal(period, pos, orien, frame, 'left_hand_motion')
        print("left hand goal sent")
        if block:
            return self.left_hand.wait()

def main():
    rospy.init_node('test_approch_object')
    rospy.sleep(1.0)

    tomm = TommClient()

    #---------------------------------------------------------------------------
    # reach the start pose

    period = 10.0

    pos = np.array([1.1, -0.5, 0.9])
    orien = np.array([0.23935, 0.66701, -0.27522, 0.64967])
    tomm.moveRightHand(period, pos, orien, 'base_footprint')
    print("1")

    pos = np.array([1.1, 0.5, 0.9])
    orien = np.array([-0.23935, 0.66701, 0.27522, 0.64967])
    tomm.moveLeftHand(period, pos, orien, 'base_footprint')
    print("2")

    rospy.spin()

if __name__ == '__main__':
    main()