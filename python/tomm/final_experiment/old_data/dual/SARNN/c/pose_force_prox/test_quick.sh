#!/bin/bash

for dir in `ls ./log/k_dim_5/pt_loss_0.01/im_decay_1000/f_decay_500`; do
    for file in `ls ./log/k_dim_5/pt_loss_0.01/im_decay_1000/f_decay_500/${dir}/*.pth`; do
        echo "python3 ./bin/test.py ${file}"
        python3 ./bin/test.py --filename ${file} --idx 0
        python3 ./bin/test.py --filename ${file} --idx 1
    done
done
