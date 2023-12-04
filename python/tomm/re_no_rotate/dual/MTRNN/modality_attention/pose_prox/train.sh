#! /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <CUDA_VISIBLE_DEVICES>"
    exit 1
fi

CUDA_VISIBLE_DEVICES=$1

python3 bin/train_image_tensorboard.py --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES