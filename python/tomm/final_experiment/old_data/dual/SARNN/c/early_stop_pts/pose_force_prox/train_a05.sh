#! /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <CUDA_VISIBLE_DEVICES>"
    exit 1
fi

CUDA_VISIBLE_DEVICES=$1

python3 bin/train_save_gif.py --image_decay_end 1000 --force_decay_end 400 --image_stop_epoch 400 --log_dir "log/k_dim_5/pt_loss_0.1/stop_400/im_decay_1000/f_decay_400" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_save_gif.py --image_decay_end 1000 --force_decay_end 400 --image_stop_epoch 200 --log_dir "log/k_dim_5/pt_loss_0.1/stop_200/im_decay_1000/f_decay_400" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_save_gif.py --image_decay_end 1000 --force_decay_end 700 --image_stop_epoch 400 --log_dir "log/k_dim_5/pt_loss_0.1/stop_400/im_decay_1000/f_decay_700" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_save_gif.py --image_decay_end 1000 --force_decay_end 700 --image_stop_epoch 200 --log_dir "log/k_dim_5/pt_loss_0.1/stop_200/im_decay_1000/f_decay_700" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_save_gif.py --image_decay_end 400 --force_decay_end 1000 --image_stop_epoch 200 --log_dir "log/k_dim_5/pt_loss_0.1/stop_200/im_decay_400/f_decay_1000" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES

python3 bin/train_save_gif.py --image_decay_end 200 --force_decay_end 1000 --image_stop_epoch 200 --log_dir "log/k_dim_5/pt_loss_0.1/stop_200/im_decay_200/f_decay_1000" --CUDA_VISIBLE_DEVICES $CUDA_VISIBLE_DEVICES