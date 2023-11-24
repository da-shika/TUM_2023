import os
import argparse
import numpy as np
import rosbag

def extract_bag_duration(bag_path):
    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time
    bag.close()
    return duration


def analyze_bag_directory(dir_path):
    durations = []
    
    sub_dirs = [d for d in os.listdir(dir_path) if os.path.isdir(os.path.join(dir_path, d))]

    if len(sub_dirs) != 0:
        for sub_dir in sub_dirs:
            sub_dir_path = os.path.join(dir_path, sub_dir)
            bag_files = [f for f in os.listdir(sub_dir_path) if f.endswith(".bag")]

            for bag_file in bag_files:
                bag_path = os.path.join(sub_dir_path, bag_file)
                duration = extract_bag_duration(bag_path)
                durations.append(duration)
                print(f"{sub_dir}/{bag_file}:{duration} seconds")
    
    else:
        dir_path = os.path.join(dir_path)
        bag_files = [f for f in os.listdir(dir_path) if f.endswith(".bag")]

        for bag_file in bag_files:
            bag_path = os.path.join(dir_path, bag_file)
            duration = extract_bag_duration(bag_path)
            durations.append(duration)
            print(f"{bag_file}:{duration} seconds")
    
    max_duration = max(durations)
    min_duration = min(durations)
    median_duration = np.median(durations)
    print("---------------------------------------------------\n")
    print(f"Max: {max_duration} seconds")
    print(f"min: {min_duration} seconds")
    print(f"medium: {median_duration} seconds")
    print(f"{len(durations)} datas are exist")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir_name", type=str, default=None)
    args = parser.parse_args()

    analyze_bag_directory(args.dir_name)

if __name__ == "__main__":
    main()