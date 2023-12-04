#! /usr/bin/env python

"""
Cannot use: set_goal_tolerance method is unknwom
"""
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import tf2_ros

class PR2CartesianControl:
    def __init__(self):
        rospy.init_node('pr2_cartesian_control')
        rospy.loginfo("pr2_cartesian_control_node start")
        
        # Create MoveGroupInterface for the PR2's right arm
        self.move_group = MoveGroupInterface("right_arm", "base_link")

        # Create a TF2 buffer and listener for transforming poses
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Allow some leeway in position (meters) and orientation (radians)
        self.move_group.set_goal_tolerance(0.01)
        #self.move_group.set_goal_position_tolerance(0.01)
        #self.move_group.set_goal_orientation_tolerance(0.1)

    def move_to_cartesian_pose(self, pose):
        # Transform the desired pose into the robot's base_link frame
        try:
            transform = self.tf_buffer.lookup_transform("base_link", pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_ros.transform(transform, pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform pose into base_link frame")
            return

        # Set the target pose
        self.move_group.set_pose_target(transformed_pose)

        # Plan and execute the motion
        plan = self.move_group.plan()
        if plan and self.move_group.execute(plan):
            rospy.loginfo("Successfully moved to Cartesian pose")
        else:
            rospy.logerr("Failed to reach Cartesian pose")

if __name__ == '__main__':
    pr2_cartesian_control = PR2CartesianControl()

    # Define the desired Cartesian pose (position and orientation)
    desired_pose = PoseStamped()
    desired_pose.header.frame_id = "base_link"  # Specify the frame of the desired pose
    desired_pose.pose.position.x = 0.6  # Desired x-coordinate in meters
    desired_pose.pose.position.y = 0.2  # Desired y-coordinate in meters
    desired_pose.pose.position.z = 1.0  # Desired z-coordinate in meters
    desired_pose.pose.orientation.w = 1.0  # Desired orientation as a quaternion (unit rotation)

    # Move the PR2 to the desired Cartesian pose
    pr2_cartesian_control.move_to_cartesian_pose(desired_pose)

    # Optionally, you can add more Cartesian poses and move the PR2 to them.