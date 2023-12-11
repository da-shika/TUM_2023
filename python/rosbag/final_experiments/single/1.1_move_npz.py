import os
import shutil
import argparse

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    args = parser.parse_args()

    bag_dir = os.path.join(args.dir, "bags")
    npz_dir = os.path.join(args.dir, "npz/single")

    move_npz_files(bag_dir, npz_dir)


if __name__ == "__main__":
    main()