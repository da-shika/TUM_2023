import numpy as np
import tf
import rospy
from geometry_msgs.msg import *
from controller_manager_msgs.srv import *

#-------------------------------------------------------------------------------
# transform messages

def to_pose_msg(pos, quad=[0,0,0,1]):
    '''convert to geometry_msgs/Pose position'''
    msg = Pose()
    msg.position.x = pos[0]
    msg.position.y = pos[1]
    msg.position.z = pos[2]
    msg.orientation.x = quad[0]
    msg.orientation.y = quad[1]
    msg.orientation.z = quad[2]
    msg.orientation.w = quad[3]
    return msg

def to_quaternion_msg(quad):
    '''convert to geometry_msgs/Quaternion'''
    msg = Quaternion()
    msg.x = quad[0]
    msg.y = quad[1]
    msg.z = quad[2]
    msg.w = quad[3]
    return msg

def to_twist_msg(vel, omega=[0,0,0]):
    '''convert to geometry_msgs/Twist velocity'''
    msg = Twist()
    msg.linear.x = vel[0]
    msg.linear.y = vel[1]
    msg.linear.z = vel[2]
    msg.angular.x = omega[0]
    msg.angular.y = omega[1]
    msg.angular.z = omega[2]
    return msg

def to_accel_msg(acc, alpha=[0,0,0]):
    '''convert to geometry_msgs/Accel velocity'''
    msg = Accel()
    msg.linear.x = acc[0]
    msg.linear.y = acc[1]
    msg.linear.z = acc[2]
    msg.angular.x = alpha[0]
    msg.angular.y = alpha[1]
    msg.angular.z = alpha[2]
    return msg

def to_wrench_msg(force, torque=[0,0,0]):
    '''convert to geometry_msgs/Wrench velocity'''
    msg = Wrench()
    msg.force.x = force[0]
    msg.force.y = force[1]
    msg.force.z = force[2]
    msg.torque.x = torque[0]
    msg.torque.y = torque[1]
    msg.torque.z = torque[2]
    return msg

def to_point_msg(point):
    '''convert to point msg'''
    msg = Point()
    msg.x = point[0]
    msg.y = point[1]
    msg.z = point[2]
    return msg

def to_vector3_msg(vec3):
    '''convert to vector3 msg'''
    msg = Vector3()
    msg.x = vec3[0]
    msg.y = vec3[1]
    msg.z = vec3[2]
    return msg

def vector3_to_np(vec):
    '''convert linear position'''
    return np.array([vec.x, vec.y, vec.z])

def pose_to_np(pose):
    '''return a np array of pos and quaternion [x,y,z,w]'''
    p = pose.position
    q = pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])

def twist_to_np(twist):
    '''return a np array of vel and omega msg'''
    v = twist.linear
    omega = twist.angular
    return np.array([v.x, v.y, v.z]), np.array([omega.x, omega.y, omega.z])

def accel_to_np(accel):
    '''return a np array of acc and alpha msg'''
    a = accel.linear
    alpha = accel.angular
    return np.array([a.x, a.y, a.z]), np.array([alpha.x, alpha.y, alpha.z])

def wrench_to_np(wrench):
    '''return a np array of force and torque msg'''
    f = wrench.force
    mu = wrench.torque
    return np.array([f.x, f.y, f.z]), np.array([mu.x, mu.y, mu.z])

def quaterion_to_np(Q):
    '''return a np array [x,y,z,w] of quaternion msg'''
    return np.array([Q.x, Q.y, Q.z, Q.w])

# ------------------------------------------------------------------------------
# usefull functions

class TransformListenerSingleton(object):
    """Retrive pose"""
    _listener = None
    @classmethod
    def init(cls):
        if cls._listener is None:
            cls._listener = tf.TransformListener()
    @classmethod
    def get(cls):
        cls.init()
        return cls._listener

def listenTransformation(parent_frame, child_frame, time_out=10.0):
    ''' lookup the transformation of child w.r.t parent '''
    try:
        listener = TransformListenerSingleton.get()
        listener.waitForTransform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(time_out))
        p, q = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        return np.array(p), np.array(q)
    except (tf.LookupException, tf.ConnectivityException) as e:
        print(e)
        return np.zeros(3), np.array([0,0,0,1])

def listActiveControllers(cntr_mng_name):
    ''' list all active controllers '''
    client = rospy.ServiceProxy(cntr_mng_name + '/list_controllers', ListControllers)
    res = client.call()
    
    activ_list = []
    for controller in res.controller:
        if controller.state == "running" and controller.claimed_resources[0].hardware_interface != "hardware_interface::JointStateInterface":
            activ_list.append(controller.name)
    return activ_list

def switchController(start_controller, cntr_mng_name):
    ''' switch to new controller, turn of the current one '''

    req = SwitchControllerRequest()
    req.strictness = 2
    req.start_controllers = [start_controller]
    req.stop_controllers = listActiveControllers(cntr_mng_name)

    client = rospy.ServiceProxy(cntr_mng_name + '/switch_controller', SwitchController)
    res = client.call(req)
    

