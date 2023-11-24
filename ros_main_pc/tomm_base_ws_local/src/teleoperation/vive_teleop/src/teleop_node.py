#! /usr/bin/env python3

import subprocess
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
import pinocchio as pin
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from visualization_msgs.msg import Marker

from behavior_msgs.srv import ChangeBehavior, ChangeBehaviorRequest, ListBehavior, ListBehaviorRequest

def transform_to_se3(transform):
    translation = transform.translation
    rotation = transform.rotation
    return pin.SE3(
        pin.Quaternion(rotation.w, rotation.x, rotation.y, rotation.z),
        np.array([translation.x, translation.y, translation.z]))

def se3_to_transform(se3):
    transform = Transform()
    transform.translation.x = se3.translation[0]
    transform.translation.y = se3.translation[1]
    transform.translation.z = se3.translation[2]
    quat = pin.Quaternion(se3.rotation)
    transform.rotation.x = quat.x
    transform.rotation.y = quat.y
    transform.rotation.z = quat.z
    transform.rotation.w = quat.w
    return transform

def pose_to_se3(pose):
    return pin.SE3(
        pin.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
        np.array([pose.position.x, pose.position.y, pose.position.z]))

def se3_to_pose(se3, use_orient=True):
    pose = Pose()
    pose.position.x = se3.translation[0]
    pose.position.y = se3.translation[1]
    pose.position.z = se3.translation[2]
    quat = pin.Quaternion(se3.rotation)
    if use_orient:
        pose.orientation.x = quat.x
        pose.orientation.y = quat.y
        pose.orientation.z = quat.z
        pose.orientation.w = quat.w
    else:
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
    return pose

def lookup_se3(tf_buffer, target_frame, source_frame, time):
    stamped_transform = tf_buffer.lookup_transform(target_frame, source_frame, time)
    return transform_to_se3(stamped_transform.transform)

def broadcast_se3(tf_broadcaster, base_frame, child_frame, se3, time):
    transform = TransformStamped()
    transform.header.stamp = time
    transform.header.frame_id = base_frame
    transform.child_frame_id = child_frame
    transform.transform = se3_to_transform(se3)
    tf_broadcaster.sendTransform(transform)

def make_sphere_marker(id, frame, r, g, b):
    marker = Marker()
    marker.header.frame_id = frame
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.type = 2
    marker.id = id
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker

def set_sphere_marker_pos(marker, pos, time):
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.header.stamp = time

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

class ViveTeleop:
    def __init__(self, log_dir):
        self.log_dir = log_dir

        # tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.sleep(1.0)

        # hardcoded offset of human to robot
        self.T_off_w = pin.SE3().Identity()
        self.T_off_w.translation = np.array([0.5, 2.1, 0.0]) # 1.4

        # msg
        self.left_out_msg = PoseStamped()
        self.right_out_msg = PoseStamped()

        # marker
        self.left_marker = make_sphere_marker(0, 'tomm/base_link', 0, 1, 0)
        self.right_marker = make_sphere_marker(1, 'tomm/base_link', 0, 1, 0)
        self.left_goal_marker = make_sphere_marker(0, 'tomm/base_link', 1, 0, 0)
        self.right_goal_marker = make_sphere_marker(1, 'tomm/base_link', 1, 0, 0)

        # input transformations
        self.T_l_wv = pin.SE3()     # left wrt vive world
        self.T_r_wv = pin.SE3()     # right wrt view world
        self.T_m_wv = pin.SE3()     # marker wrt vive world
        self.T_wv_wr = pin.SE3()    # vive world to robot world

        # flags
        self.input_cnt = 0
        self.record_loop = 1
        self.has_targets = False
        self.is_running = False
        self.is_recording = False
        self.left_buttons = None
        self.right_buttons = None

        # outputs
        self.left_teleop_pub = rospy.Publisher("/tomm/teleop/hand_link_left_pose", PoseStamped, queue_size=1)
        self.right_teleop_pub = rospy.Publisher("/tomm/teleop/hand_link_right_pose", PoseStamped, queue_size=1)    

        self.left_teleop_marker = rospy.Publisher('/left_teleop_marker', Marker, queue_size=1)
        self.right_teleop_marker = rospy.Publisher('/right_teleop_marker', Marker, queue_size=1)

        self.left_goal_marker_pub = rospy.Publisher('/left_goal_marker', Marker, queue_size=1)
        self.right_goal_marker_pub = rospy.Publisher('/right_goal_marker', Marker, queue_size=1)

        # inputs
        self.left_vive_sub = rospy.Subscriber("/vive/controller_LHR_FFDBDF42/joy", Joy, self.left_button_callback, queue_size=1)
        self.right_vive_sub = rospy.Subscriber("/vive/controller_LHR_FF5FBD43/joy", Joy, self.right_button_callback, queue_size=1)
        self.left_vive_position_sub = rospy.Subscriber("/vive/poseLeft", PoseStamped, self.left_vive_callback, queue_size=1)
        self.right_vive_position_sub = rospy.Subscriber("/vive/poseRight", PoseStamped, self.right_vive_callback, queue_size=1)

    def computeMapping(self):
        T_mv_wv = lookup_se3(self.tf_buffer, 'world_vive', 'tracker_LHR_02B7E40F', rospy.Time(0))
        T_mr_wr = lookup_se3(self.tf_buffer, 'world', 'marker', rospy.Time(0))

        T_rot = pin.SE3.Identity()
        T_rot.rotation = pin.AngleAxis(np.pi/2, np.array([0,0,1])).toRotationMatrix()
        self.T_wv_wr = T_mr_wr.act(T_rot.act(T_mv_wv.inverse()))
        
        set_sphere_marker_pos(self.left_goal_marker, np.array([1, -0.3, 1.0]), rospy.Time.now())
        set_sphere_marker_pos(self.right_goal_marker, np.array([1, +0.3, 1.0]), rospy.Time.now())

    def update(self, time):
        # are we ready?
        if not self.has_targets:
            return
        
        # transform to robot
        T_l_wr = self.T_off_w.act(self.T_wv_wr.act(self.T_l_wv))
        T_r_wr = self.T_off_w.act(self.T_wv_wr.act(self.T_r_wv))

        # visualize
        set_sphere_marker_pos(self.left_marker, T_l_wr.translation, time)
        set_sphere_marker_pos(self.right_marker, T_r_wr.translation, time)
        self.left_teleop_marker.publish(self.left_marker)
        self.right_teleop_marker.publish(self.right_marker)

        self.left_goal_marker_pub.publish(self.left_goal_marker)
        self.right_goal_marker_pub.publish(self.right_goal_marker)

        # broadcast in world
        # broadcast_se3(self.tf_broadcaster, 'world', 'vive_left', T_l_wr, time)
        # broadcast_se3(self.tf_broadcaster, 'world', 'vive_right', T_r_wr, time)

        # pub to controller
        if self.is_running:
            self.left_out_msg.pose = se3_to_pose(T_l_wr, False)
            self.right_out_msg.pose = se3_to_pose(T_r_wr, False)
            self.left_out_msg.header.stamp = time
            self.left_out_msg.header.frame_id = 'tomm/base_link'
            self.right_out_msg.header.stamp = time
            self.right_out_msg.header.frame_id = 'tomm/base_link'
            self.left_teleop_pub.publish(self.left_out_msg)
            self.right_teleop_pub.publish(self.right_out_msg)


    def left_vive_callback(self, msg): 
        self.T_l_wv = pose_to_se3(msg.pose)
        self.input_cnt += 1
        if self.input_cnt > 2:
            self.has_targets = True

    def right_vive_callback(self, msg):
        self.T_r_wv = pose_to_se3(msg.pose)
        self.input_cnt += 1
        if self.input_cnt > 2:
            self.has_targets = True

    def left_button_callback(self, msg):
        TELEOP_ON_OFF_BUTTON = 0

        if self.left_buttons is None:
            self.left_buttons = msg.buttons

        if msg.buttons[TELEOP_ON_OFF_BUTTON] == 1 and self.left_buttons[TELEOP_ON_OFF_BUTTON] == 0:
            is_on = is_teleop_running()
            if not self.is_running and is_on:
                rospy.logwarn('Teleop turning off')
                disable_teleop()
            elif not self.is_running and not is_on:
                rospy.logwarn('Teleop turning on')
                enable_teleop()

        self.left_buttons = msg.buttons


    def right_button_callback(self, msg):
        TRACKING_ON_OFF_BUTTON = 0
        ROSBAG_ON_OFF_BUTTON = 2

        if self.right_buttons is None:
            self.right_buttons = msg.buttons
        
        # check on off
        if msg.buttons[TRACKING_ON_OFF_BUTTON] == 1 and self.right_buttons[TRACKING_ON_OFF_BUTTON] == 0:
            if self.is_running:
                rospy.logwarn('Tracking turning off')
                self.is_running = False
            elif not self.is_running and is_teleop_running():
                rospy.logwarn('Tracking turning on')
                self.is_running = True

        # record operation
        rosbag_start_cmd = "rosbag record -O {0}/tutorial_{1}.bag ".format(self.log_dir, str(self.record_loop)) + \
                            "/usb_camera/republished/compressed /tomm/joint_states /tomm/teleop_left_hand/pose /tomm/teleop_right_hand/pose \
                            /tomm/arm_right/hand/right_hand_back/data /tomm/arm_right/hand/left_hand_back/data"
        rosbag_stop_cmd = "pkill -f 'rosbag record'"
        if msg.buttons[ROSBAG_ON_OFF_BUTTON] == 1 and self.right_buttons[ROSBAG_ON_OFF_BUTTON] == 0:
            if self.is_recording:
                rospy.logwarn('Recording stop')
                self.is_recording = False
                subprocess.call(rosbag_stop_cmd, shell=True) 
            elif not self.is_recording:
                rospy.logwarn('Recording start')
                self.is_recording = True
                rosbag_process = subprocess.Popen(rosbag_start_cmd, shell=True)
                self.record_loop += 1
        
        self.right_buttons = msg.buttons


def main():
    teleop = ViveTeleop('/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate/untought_box2')

    teleop.computeMapping()

    rospy.logwarn("vive_pr2_controller start")

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        teleop.update(rospy.Time.now())
        rate.sleep()

    rospy.logwarn("vive_pr2_controller done")

#-------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("vive_pr2_controller")
        main()
    except rospy.ROSInitException:
        pass