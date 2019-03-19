#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys, numpy as np
import matplotlib.pyplot as plt
import tf.transformations

import julie_misc.plot_utils as jpu
import homere_control.utils as hcu
import pdb

def weighted_sample(weights, sample_size):
    """
    Returns a weighted sample without replacement. Note: weights.dtype should be float for speed, see: http://sociograph.blogspot.nl/2011/12/gotcha-with-numpys-searchsorted.html
    """
    totals = np.cumsum(weights)
    sample = []
    for i in xrange(sample_size):
        rnd = random.random() * totals[-1]
        idx = np.searchsorted(totals,rnd,'right')
        sample.append(idx)
        totals[idx:] -= weights[idx]
    return sample


class RandomDataset():
    '''
    A dataset with uniform random wheel rotational velocities.
    '''
    def __init__(self, _nb_points, dt=0.01, enc_mu=0., enc_std=0.1):
        self.w_r, self.w_s = 0.2, 0.5
        if 0:
            self.enc_vel_lw, self.enc_vel_rw = [np.random.uniform(low=-1., high=1., size=_nb_points) for i in range(2)]
        else:
            length, angle = np.sqrt(np.random.uniform(0, 1, size=_nb_points)), np.pi * np.random.uniform(0, 2, size=_nb_points)
            self.enc_vel_lw, self.enc_vel_rw = length * np.cos(angle), length * np.sin(angle)
        self.enc_vel_stamp = np.arange(0, 0+_nb_points*dt, dt)
        err_lw, err_rw = [np.random.normal(loc=enc_mu, scale=enc_std, size=_nb_points) for i in range(2)]
        foo = np.array([hcu.kin_vel_of_wheel_vel(lrv, rrv, self.w_r, self.w_s) for lrv, rrv in zip(self.enc_vel_lw+err_lw, self.enc_vel_rw+err_rw)])
        self.truth_lvel, self.truth_lvel_body, self.truth_rvel = [np.zeros((_nb_points, 3)) for i in range(3)]
        self.truth_lvel[:,0] = self.truth_lvel_body[:,0] = foo[:,0]
        self.truth_rvel[:,2] = foo[:,1]
        self.truth_vel_stamp = self.enc_vel_stamp
        
        
    def uniformize(self):
        # https://stackoverflow.com/questions/14791918/sampling-histograms-such-that-the-sum-over-the-sample-is-uniform
        pass


        
class DataSet:

    def __init__(self, filename, _type='oscar'):
        print('## loading {}'.format(filename))
        self.data =  np.load(filename)
        self.enc_lw, self.enc_rw = self.data['encoders_lw'], self.data['encoders_rw']
        try:
            self.enc_vel_lw1, self.enc_vel_rw1 = self.data['encoders_lw_rvel'], self.data['encoders_rw_rvel']
        except KeyError:
            print('# no enc_vel')
        try:
            self.lw_pwm, self.rw_pwm = self.data['pwm_lw'], self.data['pwm_rw']
        except KeyError: pass
        try:
            self.pitch, self.pitch_dot = self.data['imu_pitch'], self.data['imu_pitch_dot']
        except KeyError: pass
        self.enc_stamp = self.data['encoders_stamp']

        if _type == 'oscar':
            try:
                self.truth_pos, self.truth_ori = self.data['mocap_pos'], self.data['mocap_ori']
                self.truth_stamp = np.array([st.to_sec() for st in self.data['mocap_stamp']])
                self.differentiate_truth_for_vel()
            except KeyError:
                self.truth_pos, self.truth_ori, self.truth_stamp = [], [], []
                self.truth_lvel, self.truth_vel_stamp = [], []
        else:
            self.truth_pos, self.truth_ori = self.data['truth_pos'], self.data['truth_ori']
            self.truth_stamp = self.data['truth_stamp']
            self.truth_lvel, self.truth_rvel = self.data['truth_lvel'], self.data['truth_rvel']
            self.truth_vel_stamp = self.truth_stamp

        print('##  found {} debug_io and {} truth'.format(len(self.enc_stamp), len(self.truth_stamp)))
        print('##  spaning {:.1f} s'.format(self.enc_stamp[-1]-self.enc_stamp[0]))
        self.differentiate_enc_for_vel()
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

        
    def differentiate_enc_for_vel(self):
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


def plot_encoders(_ds, filename=None):
    fig = jpu.prepare_fig(window_title='Encoders')
    ax = plt.subplot(4,1,1)
    plt.plot(_ds.enc_stamp, _ds.enc_lw, '.', label="lw")
    plt.plot(_ds.enc_stamp, _ds.enc_rw, '.', label="rw")   
    jpu. decorate(ax, title='Encoders position', xlab='time in s', ylab='rad', legend=True)
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
    jpu.savefig(filename)

    
def plot_encoders_stats(_ds, filename=None, title=''):
    ''' plot encoders histograms '''
    fig = jpu.prepare_fig(window_title='Encoders Stats ({})'.format(title))
    ax = plt.subplot(2,2,1)
    plt.hist(_ds.enc_vel_lw)
    jpu. decorate(ax, title='Left Wheel', xlab='rvel in rad/s', ylab='samples')
    ax = plt.subplot(2,2,2)
    plt.hist(_ds.enc_vel_lw)
    jpu. decorate(ax, title='Right Wheel', xlab='rvel in rad/s', ylab='samples')
    ax = plt.subplot(2,2,3)
    plt.hist(_ds.truth_lvel_body[:,0])
    jpu. decorate(ax, title='Linear Velocity', xlab='m/s', ylab='samples')
    ax = plt.subplot(2,2,4)
    plt.hist(_ds.truth_rvel[:,2])
    jpu. decorate(ax, title='Angular Velocity', xlab='rad/s', ylab='samples')
    jpu.savefig(filename)


def plot_encoders_3D(_ds, filename=None, title=''):
    fig = jpu.prepare_fig(window_title='Encoders 3D ({})'.format(title))
    ax = fig.add_subplot(121, projection='3d')
    truth_lvel_body_1 = interpolate(_ds.truth_lvel_body, _ds.truth_vel_stamp, _ds.enc_vel_stamp)
    ax.scatter(_ds.enc_vel_lw, _ds.enc_vel_rw, truth_lvel_body_1[:,0], s=0.1)
    ax.set_xlabel('$\omega_l (rad/s)$');ax.set_ylabel('$\omega_r (rad/s)$')
    ax.set_zlabel('$V (m/s)$')
    jpu. decorate(ax, title='Linear Velocity vs/enc vels')
    ax = fig.add_subplot(122, projection='3d')
    truth_rvel_1 = interpolate(_ds.truth_rvel, _ds.truth_vel_stamp, _ds.enc_vel_stamp)
    ax.scatter(_ds.enc_vel_lw, _ds.enc_vel_rw, truth_rvel_1[:,2], s=0.1)#, c=c, marker=m)
    ax.set_xlabel('$\omega_l (rad/s)$');ax.set_ylabel('$\omega_r (rad/s)$')
    ax.set_zlabel('$\Omega (rad/s)$')
    jpu. decorate(ax, title='Angular Velocity vs/enc vels')
    jpu.savefig(filename)
    return fig
    
    
    
def plot_pwm(_ds, filename=None):
    figsize=(20.48, 2.56)
    fig = jpu.prepare_fig(window_title='PWM', figsize=figsize)
    ax = plt.subplot(1,1,1)
    plt.plot(_ds.enc_stamp, _ds.lw_pwm, '.', label="lw")
    plt.plot(_ds.enc_stamp, _ds.rw_pwm, '.', label="rw")
    jpu. decorate(ax, title='PWM', xlab='time in s', ylab='', legend=True)

def plot_imu(_ds, filename=None):
    figsize=(20.48, 5.12)
    fig = jpu.prepare_fig(window_title='IMU', figsize=figsize)
    ax = plt.subplot(2,1,1)
    plt.plot(_ds.enc_stamp, np.rad2deg(_ds.pitch), '.', label="pitch")
    jpu. decorate(ax, title='imu pitch', xlab='time in s', ylab='deg', legend=True)
    ax = plt.subplot(2,1,2)
    plt.plot(_ds.enc_stamp, np.rad2deg(_ds.pitch_dot), '.', label="pitch_dot")
    jpu. decorate(ax, title='imu pitch dot', xlab='time in s', ylab='deg/s', legend=True)
    
    
def plot_truth_vel(_ds, filename=None):
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
    jpu.savefig(filename)
    
    
def plot2d(_ds, filename=None):
    fig = jpu.prepare_fig(window_title='2D')
    plt.plot(_ds.truth_pos[:,0], _ds.truth_pos[:,1], '.', label='truth')
    jpu. decorate(plt.gca(), title='truth 2D points', xlab='m', ylab='m', legend=True, xlim=None, ylim=None)
    plt.axis('equal')
    jpu.savefig(filename)
    return fig

def plot_all(ds):
    plot_encoders(ds)
    plot_encoders_stats(ds)
    #plot_pwm(ds)
    plot2d(ds)
    plot_truth_vel(ds)
    plot_encoders_3D(ds)
    #plot_imu(ds)
        
if __name__ == '__main__':
    #_ds = DataSet('/home/poine/work/homere/homere_control/data/odom_gazebo_2.npz', _type='homere')
    #_ds = DataSet('/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_4.npz', _type='oscar')
    #_ds = DataSet('/home/poine/work/julie/julie/julie_control/scripts/julie_odom_data_1.npz', _type='homere')
    #filename, _type = '/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_5.npz', 'homere'
    #filename, _type = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_10.npz', 'homere'
    filename, _type = '/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_04_sine_2.npz', 'oscar'
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    _ds = DataSet(filename, _type)
    #plot_encoders(_ds)
    #plot2d(_ds)
    #plot_truth_vel(_ds)
    plot_all(_ds)
    plt.show()
