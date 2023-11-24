import numpy as np
import argparse

def load_data(data_dir, train='train', data_type='left'):
    minmax = [0.1, 0.9]
    forces = np.load(data_dir + "{}/{}_forces_raw.npy".format(train, data_type))
    force_bounds = np.load(data_dir + "{}_force_bounds_raw.npy".format(data_type))
    for i in range(len(forces)):
        for k in range(forces.shape[2]):
            if max(forces[i,:,k]) < 0.25:
                forces[i,:,k] = 0
    force_bounds[1] = np.array([1e-10 if x<0.25 else x for x in force_bounds[1]])
    return forces, force_bounds

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", type=str, default=None)
    args = parser.parse_args()

    data_dir = args.dir + "/"
    left_train_force, left_force_bounds = load_data(data_dir, train='train', data_type='left')
    left_test_force, left_force_bounds = load_data(data_dir, train='test', data_type='left')
    right_train_force, right_force_bounds = load_data(data_dir, train='train', data_type='right')
    right_test_force, right_force_bounds = load_data(data_dir, train='test', data_type='right')

    np.save(data_dir + "train/left_forces.npy", left_train_force)
    np.save(data_dir + "test/left_forces.npy", left_test_force)
    np.save(data_dir + "train/right_forces.npy", right_train_force)
    np.save(data_dir + "test/right_forces.npy", right_test_force)

    np.save(data_dir + "left_force_bounds.npy", left_force_bounds)
    np.save(data_dir + "right_force_bounds.npy", right_force_bounds)

if __name__ == "__main__":
    main()
