import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

def visualize(npz_file):
    npz = np.load(npz_file)

    left_force = npz["left_forces"]
    right_force = npz["right_forces"]
    left_prox = npz["left_proximities"]
    right_prox = npz["right_proximities"]
    T = left_force.shape[0]

    plt.xlabel("T")
    plt.ylabel("value")
    plt.title("Tactile information")
    plt.subplot(2,2,1)

    def _plot(plot_area, data):
        plt.subplot(2, 2, plot_area)
        for i in range(data.shape[1]):
            plt.plot(range(T), data[:,i])
        plt.legend()

    _plot(1, left_force)
    _plot(2, right_force)
    _plot(3, left_prox)
    _plot(4, right_prox)


    for i in range(left_force.shape[1]):
        plt.plot(range(T), left_force[:,i])
        plt.plot(range(T), right_force[:,i])
        plt.plot(range(T), left_prox[:,i])
        plt.plot(range(T), right_prox[:,i])

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    parser.add_argument("--file", type=str, default=None)
    args = parser.parse_args()

    if args.file is None:
        file = os.listdir(args.dir)[0]
    else:
        file = args.file
  
    npz_file = args.dir + "/" + file

    visualize(npz_file)

    
if __name__ == "__main__":
    main()