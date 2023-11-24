import cv2
import rosbag
from cv_bridge import CvBridge

class ImageRectCheck:
    def __init__(self, bag_file, save_file):
        self.bag = rosbag.Bag(bag_file, "r")
        self.save_file = save_file
        bridge = CvBridge()
        self.is_first_img = True

        for topic, msg, _ in self.bag.read_messages():
            if topic == "/usb_camera/republished/compressed":
                if self.is_first_img == True:
                    self.image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    self.is_first_img = False

    def run(self):
        cv2.imwrite(save_file, self.image)


if __name__ == "__main__":
    dir_name = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/bags"
    file_name = "tomm_tutorial_l_001.bag"
    bag_file = dir_name + "/" + file_name
    save_file = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/initial_image.png"
    
    image_rect_check = ImageRectCheck(bag_file, save_file)
    image_rect_check.run()