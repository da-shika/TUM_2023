#! /usr/bin/env python
# # # -*-coding: utf-8 -*-

"""
Cannot use: pr2_kinematics_msgs.srv is not exist
"""

import rospy
from geometry_msgs.msg import Pose
from pr2_kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

def move_pr2_in_cartesian_space():
    # ROSノードの初期化
    rospy.init_node('move_pr2_cartesian_space')

    # 逆運動学のサービスクライアントを作成
    ik_service = rospy.ServiceProxy('/pr2_right_arm_kinematics/get_ik', GetPositionIK)

    # Joint Trajectory Actionのクライアントを作成
    joint_trajectory_client = actionlib.SimpleActionClient('/pr2/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    joint_trajectory_client.wait_for_server()

    # 目標カーテシアン座標（位置と姿勢）を設定
    target_pose = Pose()
    target_pose.position.x = 0.5  # x座標 [m]
    target_pose.position.y = 0.0  # y座標 [m]
    target_pose.position.z = 0.7  # z座標 [m]

    target_pose.orientation.x = 0.0  # 姿勢のクォータニオン
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0

    # 逆運動学のリクエストを作成
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.pose_stamped.pose = target_pose
    ik_request.ik_request.pose_stamped.header.frame_id = "base_link"
    ik_request.ik_request.ik_link_name = "r_wrist_roll_link"  # エンドエフェクタのリンク名
    ik_request.ik_request.ik_seed_state.joint_state.name = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
    ik_request.ik_request.ik_seed_state.joint_state.position = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]

    # 逆運動学の計算を実行
    try:
        ik_response = ik_service(ik_request)
        joint_positions = ik_response.solution.joint_state.position

        # 関節角度をJointTrajectoryに変換
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ik_request.ik_request.ik_seed_state.joint_state.name
        joint_trajectory.points.append(JointTrajectoryPoint(positions=joint_positions, time_from_start=rospy.Duration(2.0)))

        # Joint TrajectoryをPR2に送信
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory
        joint_trajectory_client.send_goal(goal)
        joint_trajectory_client.wait_for_result()

        rospy.loginfo("PR2 moved to target Cartesian pose.")
    except rospy.ServiceException as e:
        rospy.logerr("IK service call failed: %s", e)

if __name__ == '__main__':
    try:
        move_pr2_in_cartesian_space()
    except rospy.ROSInterruptException:
        pass
