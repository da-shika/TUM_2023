import os
import shutil
import argparse

def move_bag_files(bag_dir):
    sub_dirs = [b for b in os.listdir(bag_dir) if os.path.isdir(os.path.join(bag_dir, b))]

    for sub_dir in sub_dirs:
      sub_dir_path = os.path.join(bag_dir, sub_dir)
      bag_files = [f for f in os.listdir(sub_dir_path) if f.endswith(".bag")]

      for bag_file in bag_files:
          bag_path = os.path.join(sub_dir_path, bag_file)
          new_path = os.path.join(bag_dir, bag_file)
          shutil.move(bag_path, new_path)
          print(f"Moved: {bag_files}")

    for files in bag_files:
        bag_path = os.path.join(bag_dir, files)
        shutil.move(bag_path, files)
        

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    args = parser.parse_args()

    move_bag_files(args.dir)

if __name__ == "__main__":
    main()