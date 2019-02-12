#!/usr/bin/env python

import numpy as np, matplotlib.pyplot as plt, matplotlib
import sklearn.linear_model

import homere_control.io_dataset as hciods, homere_control.utils as hcu
import julie_misc.plot_utils as jpu

import pdb

def plot_kinematics_truth(_ds, filename=None):
    #fig = jpu.prepare_fig(window_title='Encoders 3D')
    fig = hciods.plot_encoders_3D(_ds)
    ax = fig.add_subplot(121, projection='3d')
    X = np.arange(-1, 1, 0.01)
    Y = np.arange(-1, 1, 0.01)
    X, Y = np.meshgrid(X, Y)
    lvel, rvel = np.zeros_like(X), np.zeros_like(X) 
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            lvel[i, j], rvel[i, j] = hcu.kin_vel_of_wheel_vel(X[i,j], Y[i,j], _ds.w_r, _ds.w_s)
    #pdb.set_trace()
    ax = fig.add_subplot(121, projection='3d')
    surf = ax.plot_surface(X, Y, lvel, cmap=matplotlib.cm.coolwarm,
                           linewidth=0, antialiased=True, alpha=0.5)
    ax.scatter(_ds.enc_vel_lw, _ds.enc_vel_rw, _ds.truth_lvel, s=0.1)#, c=c, marker=m)
    ax.set_xlabel('$\omega_l (rad/s)$');ax.set_ylabel('$\omega_r (rad/s)$')
    ax.set_zlabel('$V (m/s)$')
    jpu. decorate(ax, title='Linear Velocity vs/enc vels')
    
    ax = fig.add_subplot(122, projection='3d')
    surf = ax.plot_surface(X, Y, rvel, cmap=matplotlib.cm.coolwarm,
                           linewidth=0, antialiased=True, alpha=0.5)
    ax.scatter(_ds.enc_vel_lw, _ds.enc_vel_rw, _ds.truth_rvel, s=0.1)#, c=c, marker=m)
    ax.set_xlabel('$\omega_l (rad/s)$');ax.set_ylabel('$\omega_r (rad/s)$')
    ax.set_zlabel('$\Omega (rad/s)$')
    jpu. decorate(ax, title='Angular Velocity vs/enc vels')
    jpu.savefig(filename)
 
if __name__ == '__main__':
    if 1:
        name, _ds =  'random', hciods.RandomDataset(int(5e3))
    else:
        #filename, _type = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_10.npz', 'homere'
        #name, _ds = 'homere_gazebo_io_10', hciods.DataSet(filename, _type)
        #filename, _type, name = '/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_4.npz', 'oscar', 'oscar_smocap_4'
        filename, _type, name = '/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_4.npz', 'julie', 'julie_gazebo_4'
        _ds = hciods.DataSet(filename, _type)
        hciods.plot2d(_ds, '/tmp/{}_enc_2D.png'.format(name))
        plt.show()
    #hciods.plot_encoders_stats(_ds, '/tmp/{}_enc_stats.png'.format(name), name)
    #hciods.plot_encoders_3D(_ds, '/tmp/{}_enc_3D.png'.format(name), name)
    plot_kinematics_truth(_ds, '/tmp/{}_odometry_3D.png')
    plt.show()
    
