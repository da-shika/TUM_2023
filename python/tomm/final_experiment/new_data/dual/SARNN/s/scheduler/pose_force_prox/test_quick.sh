#!/bin/bash

for dir in `ls ./log`; do
    for file in `ls ./log/${dir}/*.pth`; do
        echo "python3 ./bin/test.py ${file}"
        python3 ./bin/test.py --filename ${file} --idx 18
        python3 ./bin/test.py --filename ${file} --idx 19
    done
done
