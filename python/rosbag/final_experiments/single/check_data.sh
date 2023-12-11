#!/bin/bash

for idx in {0..15}
do
  python3 ./3_check_data.py --dir /home/genki/ros/workspaces/tomm_base_ws_local/bags/re_no_rotate/data/dual --idx $idx
done