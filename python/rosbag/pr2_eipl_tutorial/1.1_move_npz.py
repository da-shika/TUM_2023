import os
import shutil

def move_npz_files(bag_dir, npz_dir):
    if not os.path.exists(npz_dir):
        os.makedirs(npz_dir)

    files = os.listdir(bag_dir)
    npz_files = [file for file in files if file.endswith(".npz")]

    for npz_file in npz_files:
        bag_path = os.path.join(bag_dir, npz_file)
        npz_path = os.path.join(npz_dir, npz_file)
        shutil.move(bag_path, npz_path)
        print(f"Moved: {npz_file}")


if __name__ == "__main__":
    root_dir = "/home/genki/ros/workspace/pr2_ws/bags/eipl_tutorial/"
    bag_dir = root_dir + "bags"
    npz_dir = root_dir + "npz"

    move_npz_files(bag_dir, npz_dir)