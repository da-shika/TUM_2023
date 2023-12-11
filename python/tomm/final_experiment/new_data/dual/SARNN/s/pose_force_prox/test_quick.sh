#!/bin/bash

for dir in `ls ./log/k_dim_5/im_decay_1000/f_decay_600`; do
    for file in `ls ./log/k_dim_5/im_decay_1000/f_decay_600/${dir}/*.pth`; do
        echo "python3 ./bin/test.py ${file}"
        python3 ./bin/test.py --filename ${file} --idx 18
        python3 ./bin/test.py --filename ${file} --idx 19
    done
done
