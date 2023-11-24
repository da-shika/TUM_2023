import numpy as np
import pinocchio as pin
from enum import Enum

# robot and controller
from gesture_generation.tsid_wrapper import DualArmManipulatorFormulation, BodyId
import gesture_generation.tomm_conf as conf
import gesture_generation.ros_utilities as ros_utils

# ROS
import rospy
import tf2_ros
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def pose_to_se3(pose):
    return pin.SE3(
        pin.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
        np.array([pose.position.x, pose.position.y, pose.position.z]))

################################################################################
# robot
################################################################################ 

class TOMM:
    def __init__(self):
        # create the task stack
        stack = DualArmManipulatorFormulation(conf, None)

        # hand tasks gains (pos+ori)
        stack.lh.motion.setKp(10*conf.kp_hand*np.array([1,1,1,1,1,1]))
        stack.lh.motion.setKd(2*np.sqrt(stack.lh.motion.Kp))
        stack.rh.motion.setKp(10*conf.kp_hand*np.array([1,1,1,1,1,1]))
        stack.rh.motion.setKd(2*np.sqrt(stack.rh.motion.Kp))
        self.stack = stack
        
        # inital state
        self.q = conf.q_home
        self.v = np.zeros_like(self.q)
        
        # frame broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # jointstate publisher
        self.joint_state_pub = rospy.Publisher('/tomm/teleop/joints', JointState, queue_size=1)
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.stamp = rospy.Time.now()
        
        # setup frames
        model = stack.model
        actuated_link_names = [link.name for link in list(model.frames) if link.type is pin.FrameType.JOINT]
        self.joint_state_msg.name = actuated_link_names    

    def setPosture(self, q):
        self.stack.setPosture(q)

    def setLeftHand(self, T_l_w):
        self.stack.bodySetReference(BodyId.LEFT_HAND, T_l_w)
        
    def setRightHand(self, T_r_w):
        self.stack.bodySetReference(BodyId.RIGHT_HAND, T_r_w)

    def leftPose(self):
        return self.stack.bodyPlacement(BodyId.LEFT_HAND)

    def rightPose(self):
        return self.stack.bodyPlacement(BodyId.RIGHT_HAND)

    def addLeftTask(self):
        self.stack.bodyAddMotion(BodyId.LEFT_HAND)

    def addRightTask(self):
        self.stack.bodyAddMotion(BodyId.RIGHT_HAND)

    def removeLeftTask(self):
        self.stack.bodyRemoveMotion(BodyId.LEFT_HAND)

    def removeRightTask(self):
        self.stack.bodyRemoveMotion(BodyId.RIGHT_HAND)
        
    def step(self, t, dt):
        # current state
        q = self.q
        v = self.v
        
        # integrate
        _, dv_sol = self.stack.update(q, v, t)
        self.q, self.v = self.stack.integrate_dv(q, v, dv_sol, dt)
    
    def publish(self, q, v, time):

        # joint state
        self.joint_state_msg.header.stamp = time
        self.joint_state_msg.position = q.tolist()
        self.joint_state_msg.velocity = v.tolist()
        self.joint_state_pub.publish(self.joint_state_msg)

        # tfs
        self.stack.broadCastFrames(self.tf_broadcaster, time)


class TOMMTeleoperation:
    def __init__(self, robot):
        self.robot = robot

        # flags
        self.is_on = False
        self.is_left_tracking = False
        self.is_right_tracking = False
        
        # real joint state
        self.real_joint_state = None

        # offsets
        self.T_off_l = pin.SE3.Identity()
        self.T_off_r = pin.SE3.Identity()

        # time
        self.time_left = rospy.Time(0)
        self.time_right = rospy.Time(0)

        # inputs
        self.toggle_teleop_srv = rospy.Service('/toogle_teleop', Empty, self._toggleTeleopHandler)
        self.left_teleop_sub = rospy.Subscriber("/tomm/teleop/hand_link_left_pose", PoseStamped, self._leftPoseCallback)
        self.right_teleop_sub = rospy.Subscriber("/tomm/teleop/hand_link_right_pose", PoseStamped, self._rightPoseCallback) 
        self.real_joint_state_sub = rospy.Subscriber("/tomm/joint_state", JointState, self._jointStateCallback)

    def waitForJointState(self, time):
        start_time = time
        while not rospy.is_shutdown() and self.real_joint_state is None:
            rospy.logwarn('Waiting for real jointstate')
            rospy.sleep(1.0)
            if (rospy.Time() - start_time).to_sec() > 20.0:
                rospy.logerr('Cant get real joint state')
                return False
            
        if self.real_joint_state is None:
            return False

        # set to real joint state
        rospy.logwarn('Got real jointstate')
        self.robot.q = self.real_joint_state
        self.robot.v = np.zeros_like(self.real_joint_state)
        self.robot.setPosture(self.real_joint_state)
        return True

    def update(self, time):
        if (time - self.time_left).to_sec() > 0.3:
            self.is_left_tracking = False
        if (time - self.time_right).to_sec() > 0.3:
            self.is_right_tracking = False

    def _leftPoseCallback(self, msg):
        T_msg_w = pose_to_se3(msg.pose)

        if self.is_on and not self.is_left_tracking:
            T_l_w = self.robot.leftPose()
            if np.linalg.norm(T_l_w.translation - T_msg_w.translation) < 0.05:
                rospy.logwarn('Left arm tracking')
                self.is_left_tracking = True
                self.T_off_l = T_msg_w.actInv(T_l_w)
                self.T_off_l.translation = np.zeros(3)

        if self.is_left_tracking:
            self.time_left = msg.header.stamp
            self.robot.setLeftHand(T_msg_w.act(self.T_off_l))
    
    def _rightPoseCallback(self, msg):
        T_msg_w = pose_to_se3(msg.pose)

        if self.is_on and not self.is_right_tracking:
            T_r_w = self.robot.rightPose()
            if np.linalg.norm(T_r_w.translation - T_msg_w.translation) < 0.05:
                rospy.logwarn('Right arm tracking')
                self.is_right_tracking = True
                self.T_off_r = T_msg_w.actInv(T_r_w)
                self.T_off_r.translation = np.zeros(3)

        if self.is_right_tracking:
            self.time_right = msg.header.stamp
            self.robot.setRightHand(T_msg_w.act(self.T_off_r))

    def _jointStateCallback(self, msg):
        if self.robot.stack.wrapper.na == len(msg.position):
            self.real_joint_state = np.array(msg.position)

    def _toggleTeleopHandler(self, req):
        if self.is_on:
            rospy.logwarn('Turn off tracking')
            self.is_on = False
            self.is_right_tracking = False
            self.is_left_tracking = False
            self.robot.removeLeftTask()
            self.robot.removeRightTask()
        else:
            rospy.logwarn('Turn on tracking')
            self.is_on = True
            self.robot.addLeftTask()
            self.robot.addRightTask()
        return EmptyResponse()


################################################################################
# main
################################################################################  
    
def main():
    robot = TOMM()
    teleop = TOMMTeleoperation(robot)
    
    # recive the first joint state
    if not teleop.waitForJointState(rospy.Time.now()):
        return
    
    ############################################################################
    dt = conf.dt
    t = 0
    
    rate = rospy.Rate(conf.f_cntr)
    while not rospy.is_shutdown():
        t += dt
        time = rospy.Time.now()
        
        teleop.update(time)
        robot.step(t, dt)
        robot.publish(robot.q, robot.v, time)
       
        rate.sleep()
            
if __name__=="__main__":
    rospy.init_node('tomm_gesture_node')
    rospy.sleep(1.0)
    main()