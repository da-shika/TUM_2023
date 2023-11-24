import cv2
import rosbag
from cv_bridge import CvBridge

class ImageRectCheck:
    def __init__(self, bag_file):
        self.bag = rosbag.Bag(bag_file, "r")
        bridge = CvBridge()
        self.is_first_img = True

        for topic, msg, _ in self.bag.read_messages():
            if topic == "/usb_camera/republished/compressed":
                if self.is_first_img == True:
                    self.image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    self.is_first_img = False

        self.click_x, self.click_y = -1, -1
        self.rectangle_height = 480
        self.rectangle_width = 480

        cv2.namedWindow("Image with Rectangle")
        cv2.setMouseCallback("Image with Rectangle", self.mouse_callback)

    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_x, self.click_y = x, y
            self.draw_rectangle()

    def draw_rectangle(self):
        if self.click_x != -1 and self.click_y != -1:
            top_left = (self.click_x, self.click_y)
            print(top_left)
            buttom_right = (self.click_x + self.rectangle_width, self.click_y + self.rectangle_height)

            image_with_rectangle = self.image.copy()
            cv2.rectangle(image_with_rectangle, top_left, buttom_right, (0,255,0), 2)
            cv2.circle(image_with_rectangle, (self.click_x, self.click_y), 3, (0,0,255), -1)
            
            cv2.imshow("Image with Rectangle", image_with_rectangle)

    def run(self):
        cv2.imshow("Image with Rectangle", self.image)
        print(self.image.shape)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.bag.close()


if __name__ == "__main__":
    dir_name = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/bags"
    file_name = "tomm_tutorial_l_001.bag"
    bag_file = dir_name + "/" + file_name
    
    image_rect_check = ImageRectCheck(bag_file)
    image_rect_check.run()