#! /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <CUDA_VISIBLE_DEVICES>"
    exit 1
fi

CUDA_VISIBLE_DEVICES=$1

python3 bin/train.py --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

cd /home/shikada/python/TUM/final_experiments/old_data/dual/SARNN/h/pose
python3 bin/train.py --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

cd /home/shikada/python/TUM/final_experiments/old_data/single/SARNN/pose_force_prox
python3 bin/train.py --k_dim 5 --log_dir log/k_dim_5 --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train.py --k_dim 10 --log_dir log/k_dim_10 --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

cd /home/shikada/python/TUM/final_experiments/old_data/dual/SARNN/c/pose