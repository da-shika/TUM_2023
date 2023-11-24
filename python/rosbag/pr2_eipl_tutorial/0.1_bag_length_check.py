import os
import numpy as np
import rosbag

def extract_bag_duration(bag_path):
    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time
    bag.close()
    return duration

def extract_after_last_slash(input_str):
    last_slash_idx = input_str[::-1].find("/")
    if last_slash_idx != -1:
        result = input_str[-last_slash_idx]
        return result
    else:
        return input_str


def analyze_bag_directory(dir_path):
    durations = []
    
    sub_dirs = [d for d in os.listdir(dir_path) if os.path.isdir(os.path.join(dir_path, d))]

    for sub_dir in sub_dirs:
        sub_dir_path = os.path.join(dir_path, sub_dir)
        bag_files = [f for f in os.listdir(sub_dir_path) if f.endswith(".bag")]

        for bag_file in bag_files:
            bag_path = os.path.join(sub_dir_path, bag_file)
            duration = extract_bag_duration(bag_path)
            durations.append(duration)
            print(f"{sub_dir}/{bag_file}:{duration} seconds")
    
    max_duration = max(durations)
    min_duration = min(durations)
    median_duration = np.median(durations)
    print(f"Max: {max_duration} seconds")
    print(f"min: {min_duration} seconds")
    print(f"medium: {median_duration} seconds")


if __name__ == "__main__":
    directory_path = "/home/genki/ros/workspace/pr2_ws/bags/eipl_tutorial"
    analyze_bag_directory(directory_path)