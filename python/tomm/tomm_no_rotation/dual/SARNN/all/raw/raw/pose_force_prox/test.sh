#!/bin/bash

for dir in `ls ./log/k_dim_5`; do
    for file in `ls ./log/k_dim_5/${dir}/*.pth`; do
        echo "python3 ./bin/test.py ${file}"
        python3 ./bin/test.py --filename ${file} --idx 3
        python3 ./bin/test.py --filename ${file} --idx 4
        python3 ./bin/test.py --filename ${file} --idx 6
        python3 ./bin/test.py --filename ${file} --idx 7
        python3 ./bin/test.py --filename ${file} --idx 9
        python3 ./bin/test.py --filename ${file} --idx 10
        python3 ./bin/test.py --filename ${file} --idx 11
    done
done
