import numpy as np

def _load_data(train='train', data_type='left'):
    detaset_folder = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/data/dual" + "/"
    minmax = [0.1, 0.9]
    forces = np.load(detaset_folder + "{}/{}_forces.npy".format(train, data_type))
    force_bounds = np.load(detaset_folder + "{}_force_bounds.npy".format(data_type))
    for i in range(len(forces)):
        for k in range(forces.shape[2]):
            if max(forces[i,:,k]) < 0.25:
                forces[i,:,k] = 0
    force_bounds[1] = np.array([1e-10 if x<0.25 else x for x in force_bounds[1]])
    return forces, force_bounds

def main():
    left_train_force, left_force_bounds = _load_data(train='train', data_type='left')
    left_test_force, left_force_bounds = _load_data(train='test', data_type='left')
    right_train_force, right_force_bounds = _load_data(train='train', data_type='right')
    right_test_force, right_force_bounds = _load_data(train='test', data_type='right')

    detaset_folder = "/home/genki/ros/workspaces/tomm_base_ws_local/bags/tutorial/data/dual" + "/"
    np.save(detaset_folder + "/train/left_forces.npy", left_train_force)
    np.save(detaset_folder + "/test/left_forces.npy", left_test_force)
    np.save(detaset_folder + "/train/right_forces.npy", right_train_force)
    np.save(detaset_folder + "/test/right_forces.npy", right_test_force)

    np.save(detaset_folder + "left_force_bounds.npy", left_force_bounds)
    np.save(detaset_folder + "right_force_bounds.npy", right_force_bounds)

if __name__ == "__main__":
    main()
