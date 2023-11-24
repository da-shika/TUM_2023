#! /usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraDisplay:
    def __init__(self):
        rospy.init_node("camera_display_node", anonymous=True)
        rospy.loginfo("camera_display_node start")
        
        self.image = None
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)

    def camera_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            scale = 2
            height, width = self.image.shape[:2]
            self.image = cv2.resize(self.image, (scale*width, scale*height))
            cv2.imshow("Image", self.image)                    
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    camera_display = CameraDisplay()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown")
        cv2.destroyAllWindows()