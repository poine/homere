#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import tf.transformations

import julie_misc.plot_utils as jpu
import homere_control.utils as hcu
import pdb

class DataSet:

    def __init__(self, filename, _type='oscar'):
        print('## loading {}'.format(filename))
        self.data =  np.load(filename)
        self.enc_lw, self.enc_rw = self.data['encoders_lw'], self.data['encoders_rw']
        self.enc_stamp = self.data['encoders_stamp']

        if _type == 'oscar':
            self.truth_pos, self.truth_ori = self.data['mocap_pos'], self.data['mocap_ori']
            self.truth_stamp = np.array([st.to_sec() for st in self.data['mocap_stamp']])
            self.differentiate_truth_for_vel()
        else:
            self.truth_pos, self.truth_ori = self.data['truth_pos'], self.data['truth_ori']
            self.truth_stamp = self.data['truth_stamp']
            self.truth_lvel, self.truth_rvel = self.data['truth_lvel'], self.data['truth_rvel']
            self.truth_vel_stamp = self.truth_stamp
        self.compute_enc_vel()
        self.compute_truth_lvel()

    def differentiate_truth_for_vel(self):
        self.truth_dt = self.truth_stamp[1:] - self.truth_stamp[:-1]
        self.truth_dp = self.truth_pos[1:] - self.truth_pos[:-1]
        self.truth_lvel = np.array([ dp / dt for dp, dt in zip(self.truth_dp, self.truth_dt)])
        self.truth_vel_stamp = (self.truth_stamp[:-1] + self.truth_stamp[1:])/2
        # FIXME: this assumes that world to body is a z-aligned axis rotation
        self.truth_yaw = np.array([tf.transformations.euler_from_quaternion(q, axes='sxyz')[2] for q in self.truth_ori])
        self.truth_rvel = np.zeros((len(self.truth_vel_stamp), 3))
        self.truth_rvel[:,2] = self.truth_yaw[1:] - self.truth_yaw[:-1]
        self.truth_rvel[:,2] /= self.truth_dt

        
    def compute_enc_vel(self):
        # encoders
        self.enc_vel_lw = self.enc_lw[1:] - self.enc_lw[:-1]
        self.enc_vel_rw = self.enc_rw[1:] - self.enc_rw[:-1]
        self.enc_dt = self.enc_stamp[1:] - self.enc_stamp[:-1]
        self.enc_vel_lw /= self.enc_dt
        self.enc_vel_rw /= self.enc_dt
        self.enc_vel_stamp = (self.enc_stamp[:-1] + self.enc_stamp[1:])/2

        self.enc_vel_sum = self.enc_vel_lw + self.enc_vel_rw
        self.enc_vel_dif = self.enc_vel_lw - self.enc_vel_rw

    def compute_truth_lvel(self):
        self.truth_ori_Tb2w = np.array([hcu.T_of_t_q(_t, _q) for _t, _q in zip(self.truth_pos, self.truth_ori)])
        self.truth_lvel_body = np.array([np.dot(Tb2w[:3,:3].T, lvel_w) for Tb2w, lvel_w in zip(self.truth_ori_Tb2w, self.truth_lvel)])
        #self.truth_norm_vel = np.linalg.norm(self.truth_lvel, axis=1)


def interpolate(_d, _d_stamps, _stamps_1):
    # returns an interpolated version of _d at stamps  _stamps_1
    _d_1 = np.zeros((len(_stamps_1), _d.shape[1]))
    i = -1
    for i1, s1 in enumerate(_stamps_1):
        if i < len(_d_stamps)-1 and s1 >= _d_stamps[i+1]:
            i+=1
        if i==-1:
            _d_1[i1] = _d[0]
        elif i < len(_d_stamps)-1:
            _d_1[i1] = (s1-_d_stamps[i])/(_d_stamps[i+1]-_d_stamps[i])*(_d[i+1]-_d[i]) + _d[i]
        else:
            _d_1[i1] = _d[-1]
    return _d_1


def plot_interp(_d, _stamps, _d_1, _stamps_1):
    plt.plot(_stamps, _d, '.', label='orig')
    plt.plot(_stamps_1, _d_1, '.', label='interp')
    jpu. decorate(plt.gca(), title='Interp', xlab='time in s', ylab='', legend=True, xlim=None, ylim=None)


def plot_encoders(_ds):
    fig = jpu.prepare_fig(window_title='Encoders')
    ax = plt.subplot(4,1,1)
    plt.plot(_ds.enc_stamp, _ds.enc_lw, '.')
    plt.plot(_ds.enc_stamp, _ds.enc_rw, '.')   
    jpu. decorate(ax, title='Encoders position', xlab='time in s', ylab='rad', legend=None, xlim=None, ylim=None)
    ax = plt.subplot(4,1,2)
    plt.plot(_ds.enc_vel_stamp, _ds.enc_vel_lw, '.')
    plt.plot(_ds.enc_vel_stamp, _ds.enc_vel_rw, '.')   
    jpu. decorate(ax, title='Encoders velocity', xlab='time in s', ylab='rad/s', legend=None, xlim=None, ylim=None)
    ax = plt.subplot(4,1,3)
    plt.plot(_ds.enc_vel_stamp, _ds.enc_vel_sum, '.')
    jpu. decorate(ax, title='Encoders velocity sum', xlab='time in s', ylab='rad/s', legend=None, xlim=None, ylim=None)
    ax = plt.subplot(4,1,4)
    plt.plot(_ds.enc_vel_stamp, _ds.enc_vel_dif, '.')
    jpu. decorate(ax, title='Encoders velocity dif', xlab='time in s', ylab='rad/s', legend=None, xlim=None, ylim=None)
     

def plot_truth(_ds):
    fig = jpu.prepare_fig(window_title='Truth')
    ax = plt.subplot(2,1,1)
    #plt.plot(_ds.truth_stamp, _ds.truth_norm_vel, '.')
    plt.plot(_ds.truth_vel_stamp, _ds.truth_lvel_body[:,0], '.', label='x')
    plt.plot(_ds.truth_vel_stamp, _ds.truth_lvel_body[:,1], '.', label='y')
    plt.plot(_ds.truth_vel_stamp, _ds.truth_lvel_body[:,2], '.', label='z')
    jpu. decorate(ax, title='Truth velocity', xlab='time in s', ylab='m/s', legend=True, xlim=None, ylim=None)
    ax = plt.subplot(2,1,2)
    plt.plot(_ds.truth_vel_stamp, _ds.truth_rvel[:,0], '.', label='p')
    plt.plot(_ds.truth_vel_stamp, _ds.truth_rvel[:,1], '.', label='q')
    plt.plot(_ds.truth_vel_stamp, _ds.truth_rvel[:,2], '.', label='r')
    jpu. decorate(ax, title='Truth rotational velocity', xlab='time in s', ylab='rad/s', legend=True, xlim=None, ylim=[-2, 2])
    
    
    
def plot2d(_ds):
    fig = jpu.prepare_fig(window_title='2D')
    plt.plot(_ds.truth_pos[:,0], _ds.truth_pos[:,1], '.', label='truth')
    jpu. decorate(plt.gca(), title='truth 2D points', xlab='m', ylab='m', legend=True, xlim=None, ylim=None)
    plt.axis('equal')

    return fig

        



if __name__ == '__main__':
    #_ds = DataSet('/home/poine/work/homere/homere_control/data/odom_gazebo_2.npz', _type='homere')
    #_ds = DataSet('/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_4.npz', _type='oscar')
    _ds = DataSet('/home/poine/work/julie/julie/julie_control/scripts/julie_odom_data_1.npz', _type='homere')

    #plot_encoders(_ds)
    #plot2d(_ds)
    plot_truth(_ds)
    plt.show()
