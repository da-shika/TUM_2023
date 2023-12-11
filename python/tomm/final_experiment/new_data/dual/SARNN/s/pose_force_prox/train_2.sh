#! /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <CUDA_VISIBLE_DEVICES>"
    exit 1
fi

CUDA_VISIBLE_DEVICES=$1

python3 bin/train_image_tensorboard.py --image_decay_end 200 --log_dir "log/k_dim_5/im_decay_200" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --image_decay_end 600 --log_dir "log/k_dim_5/im_decay_600" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES