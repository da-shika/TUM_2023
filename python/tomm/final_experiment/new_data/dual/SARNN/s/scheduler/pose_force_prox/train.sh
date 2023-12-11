#! /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <CUDA_VISIBLE_DEVICES>"
    exit 1
fi

CUDA_VISIBLE_DEVICES=$1

python3 bin/train.py --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

cd /home/shikada/python/TUM/final_experiments/new_data/dual/SARNN/sh/scheduler/pose_force_prox
python3 bin/train.py --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

cd /home/shikada/python/TUM/final_experiments/new_data/dual/SARNN/s/scheduler/pose_force_prox