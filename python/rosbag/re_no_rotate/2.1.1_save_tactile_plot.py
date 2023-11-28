import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

def save(dir):
    for filename in os.listdir(dir):
        if filename.endswith(".npz"):
            filepath = os.path.join(dir, filename)
    
            npz = np.load(filepath)

            left_force = npz["left_forces"]
            right_force = npz["right_forces"]
            left_prox = npz["left_proximities"]
            right_prox = npz["right_proximities"]
            T = left_force.shape[0]

            print('l_shape=', left_prox.shape)
            print('r_shape=', right_prox.shape)

            plt.xlabel("T")
            plt.ylabel("value")
            plt.title("Tactile information")
            plt.subplot(2,2,1)

            def _plot(plot_area, data, title):
                plt.subplot(2, 2, plot_area)
                for i in range(data.shape[1]):
                    plt.plot(range(T), data[:,i])
                plt.legend()
                plt.title(title)
            _plot(1, left_force, "left_force")
            _plot(2, right_force, "right_force")
            _plot(3, left_prox, "left_prox")
            _plot(4, right_prox, "right_prox")

            plt.tight_layout()

            output_dir = os.path.join(dir, "output")
            output_filepath = os.path.join(output_dir, filename.replace(".npz", ".png"))
            plt.savefig(output_filepath)

            plt.clf()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    args = parser.parse_args()

    save(args.dir)

    
if __name__ == "__main__":
    main()