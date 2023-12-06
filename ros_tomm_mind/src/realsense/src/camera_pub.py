#! /usr/bin/env python

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

def publish_usb_camera():
  rospy.init_node("usb_camera_pub_node", anonymous=True)
  rospy.loginfo("usb_camera_pub_node start")

  video_device = 0
  cap = cv2.VideoCapture(video_device, cv2.CAP_V4L2)

  image_pub = rospy.Publisher("usb_camera/image_raw", Image, queue_size=10)
  bridge = CvBridge()

  rate = rospy.Rate(100)

  while not rospy.is_shutdown():
    ret, frame = cap.read()

    if ret:
      image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
      image_pub.publish(image_msg)

    rate.sleep()

  cap.release()


if __name__ == "__main__":
  try:
    publish_usb_camera()
  except rospy.ROSInterruptException:
    pass