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
    root_dir = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial"
    bag_dir = os.path.join(root_dir, "bags")
    npz_dir = os.path.join(root_dir, "npz/single")

    move_npz_files(bag_dir, npz_dir)