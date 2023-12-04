#! /usr/bin/env python

import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveTorso:
    def __init__(self):
        rospy.init_node("move_torso_node")
        rospy.loginfo("move_torso_node start")

        self.torso_joint_name = ["torso_lift_joint"]
        self.torso_pub = rospy.Publisher("/torso_controller/command", JointTrajectory, queue_size=10)

    def create_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.torso_joint_name
        header = Header()
        header.stamp = rospy.Time.now()
        trajectory.header = header
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(2.0)
        trajectory.points.append(point)
        return trajectory

    def run(self):
        rospy.logwarn("Start moving")
        trajectory = self.create_trajectory()
        trajectory.points[0].positions = [0.05]
        self.torso_pub.publish(trajectory)
        rospy.sleep(2.0)
        rospy.logwarn("Finish moving")

if __name__ == "__main__":
    move_torso = MoveTorso()
    rospy.sleep(0.5)
    move_torso.run()