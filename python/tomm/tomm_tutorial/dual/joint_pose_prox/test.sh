#!/bin/bash

for dir in `ls ./log/k_dim_5`; do
    for file in `ls ./log/k_dim_5/${dir}/*.pth`; do
        echo "python3 ./bin/test.py ${file}"
        python3 ./bin/test.py --filename ${file} --idx 0
        python3 ./bin/test.py --filename ${file} --idx 1
        python3 ./bin/test.py --filename ${file} --idx 2
        python3 ./bin/test.py --filename ${file} --idx 3
        python3 ./bin/test.py --filename ${file} --idx 4
    done
done
