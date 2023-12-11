#!/bin/bash

for dir in `ls ./log/k_dim_10`; do
    for file in `ls ./log/k_dim_10/${dir}/*.pth`; do
        echo "python3 ./bin/test.py ${file}"
        python3 ./bin/test.py --filename ${file} --idx 0
        python3 ./bin/test.py --filename ${file} --idx 1
    done
done
