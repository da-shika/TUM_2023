#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

def move_pr2_arm():
    # ROSノードの初期化
    rospy.init_node('pr2_ik_solver')
    
    # MoveIt!コマンダーの初期化
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("right_arm")  # PR2の右腕を制御する例

    # 目標位置姿勢を指定
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # 基準フレームを指定
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.7
    target_pose.pose.orientation.w = 1.0

    # 逆運動学の計算
    arm_group.set_pose_target(target_pose)
    arm_group.set_start_state_to_current_state()
    plan = arm_group.plan()

    # 計画を実行
    arm_group.execute(plan)

if __name__ == '__main__':
    try:
        move_pr2_arm()
    except rospy.ROSInterruptException:
        pass
