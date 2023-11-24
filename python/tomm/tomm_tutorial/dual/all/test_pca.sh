#!/bin/bash

for dir in `ls ./log/k_dim_5`; do
    for file in `ls ./log/k_dim_5/${dir}/*.pth`; do
        echo "python3 ./bin/test_pca_sarnn.py ${file}"
        python3 ./bin/test_pca_sarnn.py --filename ${file}
    done
done