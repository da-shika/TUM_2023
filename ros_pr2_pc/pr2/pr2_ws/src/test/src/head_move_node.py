#! /usr/bin/env python

import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveHead:
    def __init__(self):
        rospy.init_node("move_head_node")
        rospy.loginfo("move_head_node start")

        self.head_joint_name = ["head_tilt_joint", "head_pan_joint"]
        self.head_pub = rospy.Publisher("/head_traj_controller/command", JointTrajectory, queue_size=10)

    def create_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.head_joint_name
        header = Header()
        header.stamp = rospy.Time.now()
        trajectory.header = header
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(2.0)
        trajectory.points.append(point)
        return trajectory

    def run(self):
        rospy.logwarn("Start moving")
        head_trajectory = self.create_trajectory()
        head_trajectory.points[0].positions = [0.75, 0.0]
        self.head_pub.publish(head_trajectory)
        rospy.sleep(2.0)
        rospy.logwarn("Finish moving")

if __name__ == "__main__":
    move_head = MoveHead()
    rospy.sleep(0.5)
    move_head.run()