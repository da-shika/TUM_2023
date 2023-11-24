#!/usr/bin/env python3

"""
send rosbag command by python
"""

import subprocess
import time

rosbag_start_cmd = "rosbag record /vive/controller_LHR_9F5D6499/joy"
rosbag_stop_cmd = "pkill -f 'rosbag record'"

rosbag_process = subprocess.Popen(rosbag_start_cmd, shell=True)

time.sleep(10)
subprocess.call(rosbag_stop_cmd, shell=True)