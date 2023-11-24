#! /usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose

from behavior_msgs.srv import ChangeBehavior, ChangeBehaviorRequest, ListBehavior, ListBehaviorRequest

def is_teleop_running(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/list_behavior', ListBehavior)
    req = srv_client.call(ListBehaviorRequest())
    return name in req.running_behaviors
    
def disable_teleop(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/change_behavior', ChangeBehavior)
    req = ChangeBehaviorRequest()
    req.start_behaviors = []
    req.stop_behaviors = [name]
    srv_client.call(req)

def enable_teleop(name = 'teleop'):
    srv_client = rospy.ServiceProxy('/tomm/change_behavior', ChangeBehavior)
    req = ChangeBehaviorRequest()
    req.start_behaviors = [name]
    req.stop_behaviors = []
    srv_client.call(req)


class RosbagCheck:
    def __init__(self, hz, file_name):
        npz = np.load(file_name)
        self.left_pose = npz["left_poses"]
        self.right_pose = npz["right_poses"]

        self.hz = hz
        self.left_pub_msg, self.right_pub_msg = PoseStamped(), PoseStamped()

        self.left_pose_pub = rospy.Publisher("/tomm/teleop/hand_link_left_pose", PoseStamped, queue_size=10)
        self.right_pose_pub = rospy.Publisher("/tomm/teleop/hand_link_right_pose", PoseStamped, queue_size=10)

        while self.left_pose_pub.get_num_connections() == 0:
            rospy.sleep(1)
        while self.right_pose_pub.get_num_connections() == 0:
            rospy.sleep(1)
  
  #---------------------------------------------------------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.hz)
        
        # turn on teleop
        rospy.loginfo('starting teleop')
        enable_teleop()
        rospy.sleep(2.0)

        if not is_teleop_running():
            rospy.logerr('error: could not turn on teleoperation')
            return

        rospy.loginfo("start the motion")
        for i in range(len(self.left_pose)):
            self.left_pub_msg.pose= self.npy_2_pose(self.left_pose[i])
            self.left_pub_msg.header.stamp = rospy.Time.now()
            self.left_pub_msg.header.frame_id = 'tomm/base_link'
            self.left_pose_pub.publish(self.left_pub_msg)

            self.right_pub_msg.pose= self.npy_2_pose(self.right_pose[i])
            self.right_pub_msg.header.stamp = rospy.Time.now()
            self.right_pub_msg.header.frame_id = 'tomm/base_link'
            self.right_pose_pub.publish(self.right_pub_msg)

            rate.sleep()

        rospy.loginfo("stopping teleop")
        disable_teleop()

  #---------------------------------------------------------------------------------------------------------------------
    def npy_2_pose(self, npy_data):
        pose = Pose()
        pose.position.x = npy_data[0]
        pose.position.y = npy_data[1]
        pose.position.z = npy_data[2]
        pose.orientation.x = npy_data[3]
        pose.orientation.y = npy_data[4]
        pose.orientation.z = npy_data[5]
        pose.orientation.w = npy_data[6]
        return pose
    
 #---------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node("bag_pose_check_node")
    rospy.loginfo("bag_pose_check_node start")

    dir_name = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/npz/dual" + "/"
    bag_name = "tomm_tutorial_l_001.npz"
    file_name = dir_name + bag_name
    
    try:
        rosbag_check = RosbagCheck(hz=10, file_name=file_name)
        rosbag_check.run()
    except rospy.ROSInitException:
        print('error!!!')
        pass
    
if __name__ == "__main__":
    main()