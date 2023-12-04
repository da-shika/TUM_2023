#!/bin/bash

for dir in `ls ./log/k_dim_10/1e-4`; do
    for file in `ls ./log/k_dim_10/1e-4/${dir}/*.pth`; do
        echo "python3 ./bin/test.py ${file}"
        python3 ./bin/test.py --filename ${file} --idx 1
    done
done
