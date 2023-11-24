#! /usr/bin/env python3

import rospy 
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
capture_image = False

def image_callback(msg):
    img_file = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate/initial_image.png"

    try:
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        scale = 2
        height, width = image.shape[:2]
        image = cv2.resize(image, (scale*width, scale*height))
    except CvBridgeError:
        rospy.logerr("CvBridge Error:")

    if capture_image:
        cv2.imwrite(img_file, image)
    else:
        inital_image = cv2.imread(img_file)
        height, width = inital_image.shape[:2]
        inital_image = cv2.resize(inital_image, (scale*width, scale*height))
        image = cv2.addWeighted(inital_image, 0.3, image, 0.7, 0)

    cv2.imshow("image",image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('alignment_node', anonymous=True)
    rospy.loginfo("alignment_node start")
    rospy.sleep(1.0)

    image_sub = rospy.Subscriber("/usb_camera/image_raw", Image, image_callback)
    cv2.namedWindow("image", 1)

    rospy.spin()