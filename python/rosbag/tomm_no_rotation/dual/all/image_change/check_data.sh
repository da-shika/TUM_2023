#!/bin/bash

for idx in {0..12}
do
  python3 ./3_check_data.py --dir /home/genki/ros/workspaces/tomm_base_ws_local/bags/pick_no_rotate/data/dual/all --idx $idx
done