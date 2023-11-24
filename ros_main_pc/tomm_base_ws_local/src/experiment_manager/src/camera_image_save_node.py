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
        self.image_clicked = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_camera/republished/compressed", Image, self.camera_callback)

        cv2.namedWindow("Image")
        cv2.setMouseCallback("Image", self.on_mouse_click)
        
#-------------------------------------------------------------------------------
    def camera_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            scale = 2
            height, width = self.image.shape[:2]
            self.image = cv2.resize(self.image, (scale*width, scale*height))
            cv2.imshow("Image", self.image)
            cv2.waitKey(1)

            if self.image_clicked is not None:
                cv2.imwrite("initial_image.png", self.image)
                rospy.logwarn("Save image")
                self.image_clicked = None

        except CvBridgeError as e:
            print(e)


    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.image_clicked = (x, y)

#-------------------------------------------------------------------------------
if __name__ == "__main__":
    camera_display = CameraDisplay()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown")
        cv2.destroyAllWindows()