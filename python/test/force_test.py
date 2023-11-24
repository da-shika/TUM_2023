import numpy as np
import ipdb
import matplotlib.pylab as plt
import matplotlib.animation as anim
from eipl.utils import EarlyStopping, check_args, set_logdir, normalization, resize_img

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
    forces = normalization(forces, force_bounds, minmax)
    return forces



def main():
    train_left_force = _load_data(train='train', data_type='left')
    print('Left Force: shape={}, min={:.3g}, max={:.3g}'.format(train_left_force.shape, train_left_force.min(), train_left_force.max()) )
    N = train_left_force.shape[1]

    fig, ax = plt.subplots(1, 2, figsize=(10, 5), dpi=60)
    def anim_update(i):
        for k in range(2):
            ax[k].cla()
        def plot_data(ax_area, ylim_range, data, title):
            ax_area.set_ylim(ylim_range)
            ax_area.set_xlim(0, N)
            ax_area.plot(data[0], linestyle='dashed', c='k')
            for joint_idx in range(data.shape[2]):
                ax_area.plot(np.arange(i+1), data[0,:i+1, joint_idx])
            ax_area.set_xlabel('Step')
            ax_area.set_title(title)

        plot_data(ax[0], (0.0, 2.0), train_force, 'Left Forces')

    ani = anim.FuncAnimation(fig, anim_update, interval=int(N / 10), frames=N)
    output_dir = "/home/genki/temp/"
    ani.save(output_dir + "check_data_{}.gif".format(0))


if __name__ == "__main__":
    main()