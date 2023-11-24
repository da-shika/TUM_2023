import argparse
import os
import cv2
import rosbag
from cv_bridge import CvBridge

class ImageSaver:
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
        cv2.imwrite(self.save_file, self.image)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    args = parser.parse_args()

    files = os.listdir(args.dir)
    bag_file = args.dir + "/" + files[0]
    
    save_dir = os.path.dirname(args.dir)
    save_file = save_dir + "/initial_image.png"

    image_rect_check = ImageSaver(bag_file, save_file)
    image_rect_check.run()


if __name__ == "__main__":
    main()