#!/bin/bash

for idx in {0..34}
do
  python3 ./3_check_data.py --dir /home/genki/ros/workspaces/tomm_base_ws_local/bags/final_experiment/data/dual --idx $idx
done