#!/bin/bash

for dir in `ls ./log/k_dim_5/pt_loss_0.05/im_decay_1000/f_decay_1000`; do
    for file in `ls ./log/k_dim_5/pt_loss_0.05/im_decay_1000/f_decay_1000/${dir}/*.pth`; do
        echo "python3 ./bin/test_pca_sarnn.py ${file}"
        python3 ./bin/test_pca_sarnn.py --filename ${file}
    done
done