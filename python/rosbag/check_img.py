import numpy as np
import cv2

def main():
    npz = np.load("/home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate/npz/dual/tomm_tutorial_r_small_001.npz")
    left_image = npz["left_images"]

    cv2.imshow("left_image", left_image[0])
    cv2.waitKey(0)

if __name__ == "__main__":
    main()