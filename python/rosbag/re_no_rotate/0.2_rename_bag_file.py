import os
import argparse
import shutil

def rename_bags(dir_path):
    sub_dirs = [d for d in os.listdir(dir_path) if os.path.isdir(os.path.join(dir_path, d))]

    for sub_dir in sub_dirs:
        sub_dir_path = os.path.join(dir_path, sub_dir)

        bag_files = [f for f in os.listdir(sub_dir_path) if f.endswith(".bag")]
        bag_files.sort()

        for i, old_name in enumerate(bag_files):
            new_name = f"tomm_tutorial_{sub_dir}_{i+1:03d}.bag"
            old_path = os.path.join(sub_dir_path, old_name)
            new_path = os.path.join(sub_dir_path, new_name)
            shutil.move(old_path, new_path)
            print(f"Renamed: {old_name} -> {new_name}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    args = parser.parse_args()

    rename_bags(args.dir)
    
if __name__ == "__main__":
    main()