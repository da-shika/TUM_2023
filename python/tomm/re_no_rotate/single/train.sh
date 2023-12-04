#! /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <CUDA_VISIBLE_DEVICES>"
    exit 1
fi

CUDA_VISIBLE_DEVICES=$1

python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 1e-4 --log_dir "log/k_dim_5/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 1e-3 --log_dir "log/k_dim_5/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 1e-5 --log_dir "log/k_dim_5/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 1e-4 --log_dir "log/k_dim_6/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 1e-3 --log_dir "log/k_dim_6/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 1e-5 --log_dir "log/k_dim_6/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 1e-4 --log_dir "log/k_dim_7/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 1e-3 --log_dir "log/k_dim_7/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 1e-5 --log_dir "log/k_dim_7/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 1e-4 --log_dir "log/k_dim_8/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 1e-3 --log_dir "log/k_dim_8/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 1e-5 --log_dir "log/k_dim_8/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 5e-4 --log_dir "log/k_dim_5/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 5e-5 --log_dir "log/k_dim_5/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 5e-4 --log_dir "log/k_dim_6/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 5e-5 --log_dir "log/k_dim_6/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 5e-4 --log_dir "log/k_dim_7/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 5e-5 --log_dir "log/k_dim_7/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 5e-4 --log_dir "log/k_dim_8/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 5e-5 --log_dir "log/k_dim_8/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES


python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 1e-4 --log_dir "log/k_dim_5/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 1e-3 --log_dir "log/k_dim_5/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 1e-5 --log_dir "log/k_dim_5/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 1e-4 --log_dir "log/k_dim_6/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 1e-3 --log_dir "log/k_dim_6/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 1e-5 --log_dir "log/k_dim_6/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 1e-4 --log_dir "log/k_dim_7/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 1e-3 --log_dir "log/k_dim_7/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 1e-5 --log_dir "log/k_dim_7/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 1e-4 --log_dir "log/k_dim_8/1e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 1e-3 --log_dir "log/k_dim_8/1e-3" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 1e-5 --log_dir "log/k_dim_8/1e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 5e-4 --log_dir "log/k_dim_5/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 5 --temperature 5e-5 --log_dir "log/k_dim_5/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 5e-4 --log_dir "log/k_dim_6/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 6 --temperature 5e-5 --log_dir "log/k_dim_6/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 5e-4 --log_dir "log/k_dim_7/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 7 --temperature 5e-5 --log_dir "log/k_dim_7/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 5e-4 --log_dir "log/k_dim_8/5e-4" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES
python3 bin/train_image_tensorboard.py --k_dim 8 --temperature 5e-5 --log_dir "log/k_dim_8/5e-5" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES