import argparse
import os
import cv2
import rosbag
from cv_bridge import CvBridge

class ImageRectCheck:
    def __init__(self, bag_file):
        self.bag = rosbag.Bag(bag_file, "r")
        self.bridge = CvBridge()
        self.image_list = []

        self.read_images()

        self.click_x, self.click_y = -1, -1
        self.rectangle_height = 320
        self.rectangle_width = self.rectangle_height * 2
        
        cv2.namedWindow("Image with Rectangle")
        cv2.setMouseCallback("Image with Rectangle", self.mouse_callback)
    
    def read_images(self):
        cnt = 0
        for topic, msg, _ in self.bag.read_messages():
            if topic == "/usb_camera/republished/compressed":
                cnt += 1
                if cnt == 1:
                    image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    self.image_list.append(image)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_x, self.click_y = x, y
            self.draw_rectangle()

    def draw_rectangle(self):
        if self.click_x != -1 and self.click_y != -1:
            top_left = (self.click_x, self.click_y)
            bottom_right = (self.click_x + self.rectangle_width, self.click_y + self.rectangle_height)

            image_with_rectangle = self.image_list[0].copy()
            cv2.rectangle(image_with_rectangle, top_left, bottom_right, (0, 255, 0), 2)
            cv2.circle(image_with_rectangle, (self.click_x, self.click_y), 3, (0, 0, 255), -1)
            cv2.imshow("Image with Rectangle", image_with_rectangle)

    def run(self):
        cv2.imshow("Image with Rectangle", self.image_list[0])
        print(self.image_list[0].shape)
        key = cv2.waitKey(0)
        if key == ord('p'):
            for image in self.image_list:
                cv2.imshow("Image with Rectangle", image)
                cv2.waitKey(200)
        cv2.destroyAllWindows()
        self.bag.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    args = parser.parse_args()
    if args.dir is None:
        print("ディレクトリが指定されていません。")
        return
    files = os.listdir(args.dir)
    for file in files:
        if file.endswith(".bag"):
            bag_file = os.path.join(args.dir, file)
            image_rect_check = ImageRectCheck(bag_file)
            image_rect_check.run()

            
if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()