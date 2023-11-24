#! /usr/bin/env python3

import rospy 
import pinocchio as pin
import tf2_ros
import numpy as np




def transform_to_se3(transform):
    translation = transform.translation
    rotation = transform.rotation
    return pin.SE3(
        pin.Quaternion(rotation.w, rotation.x, rotation.y, rotation.z),
        np.array([translation.x, translation.y, translation.z]))

def lookup_transformation(tf_buffer, target_frame, source_frame, time):
    stamped_transform = tf_buffer.lookup_transform(target_frame, source_frame, time)
    return transform_to_se3(stamped_transform.transform)

def main():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    
    T_mv_wv = lookup_transformation(tf_buffer, 'world_vive', 'tracker_LHR_02B7E40F', rospy.Time(0))
    T_mr_wr = lookup_transformation(tf_buffer, 'world', 'marker', rospy.Time(0))

    # transformation between worlds
    T_wv_wr = T_mr_wr.act(T_mv_wv.inverse())
    print('T_wv_wr')
    print(T_wv_wr)

    print('T_wr_wv')
    print(T_wv_wr.inverse())   

if __name__ == "__main__":
    try:
        rospy.init_node("vive_pr2_controller")
        main()
    except rospy.ROSInitException:
        pass
