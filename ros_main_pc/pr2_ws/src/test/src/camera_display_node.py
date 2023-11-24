#! /usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraDisplay:
    def __init__(self):
        rospy.init_node("camera_display_node", anonymous=True)
        rospy.loginfo("camera_display_node start")
        
        self.left_image = None
        self.right_image = None
        self.both_image = None
        self.bridge = CvBridge()

        self.left_image_sub = rospy.Subscriber("/wide_stereo/left/image_raw", Image, self.left_camera_callback)
        self.right_image_sub = rospy.Subscriber("/wide_stereo/right/image_raw", Image, self.right_camera_callback)


    def left_camera_callback(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
    def right_camera_callback(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def combined_images(self):
        try:
            self.both_image = cv2.hconcat([self.left_image, self.right_image])
        except CvBridgeError as e:
            print(e)

#------------------------------------------------------------------------------------------------    
    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.left_image is not None and self.right_image is not None:
                    self.combined_images()

                    cv2.imshow("Left Image", self.left_image)
                    #cv2.imshow("Right Image", self.right_image)
                    #cv2.imshow("Both Image", self.both_image)
                    
                    cv2.waitKey(1)
        except KeyboardInterrupt:
            print("shutdown")
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        camera_display = CameraDisplay()
        camera_display.run()
    except rospy.ROSInterruptException:
        pass