#!/usr/bin/env python

import numpy as np, matplotlib.pyplot as plt
import tf.transformations
import pdb

import julie_misc.plot_utils as jpu
import homere_control.utils as hcu
import homere_control.io_dataset as hio, fit_odometry as fod

#
# original C++ implementation
#
import cpp_homere_control

#
# simplified implementation of ros diff drive controller odometry
#
class Odometer:
    def __init__(self, ws, lwr, rwr):
        self.ws, self.lwr, self.rwr = ws, lwr, rwr
    
    def start(self, pos0, yaw0, lw_angle, rw_angle, t0):
        self.lw_old_dep = lw_angle * self.lwr
        self.rw_old_dep = rw_angle * self.rwr
        self.pos, self.yaw = pos0, yaw0
        print('reset at {} {}'.format(pos0, yaw0))
        return self.pos, self.yaw
        
    def update(self, lw_angle, rw_angle, dt, i):
        #if i==1:
        #    pdb.set_trace()
        lw_cur_dep = lw_angle * self.lwr
        rw_cur_dep = rw_angle * self.rwr
        # Estimate velocity of wheels using old and current position:
        self.lw_est_vel_ = lw_cur_dep - self.lw_old_dep;
        self.rw_est_vel_ = rw_cur_dep - self.rw_old_dep;
        # Update old position with current:
        self.lw_old_dep = lw_cur_dep;
        self.rw_old_dep = rw_cur_dep;
        # Compute linear and angular diff:
        self.linear  = (self.rw_est_vel_ + self.lw_est_vel_) * 0.5 ;
        self.angular = (self.rw_est_vel_ - self.lw_est_vel_) / self.ws;

        if False:#np.abs(self.angular) > 1e-6:
            prev_yaw, r = self.yaw, self.linear/self.angular
            self.yaw += self.angular
            self.pos[0] +=  r * (np.sin(self.yaw) - np.sin(prev_yaw))
            self.pos[1] += -r * (np.cos(self.yaw) - np.cos(prev_yaw))
        else:
            #print('rk2')
            direction = self.yaw + self.angular * 0.5;
            self.pos[0] += self.linear * np.cos(direction);
            self.pos[1] += self.linear * np.sin(direction);
            self.yaw += self.angular
        if np.isnan(self.pos).any():
            print('#####naaan')
        return self.pos, self.yaw



def run_on_ds(odom, ds, label, start_idx=0, run_len=None):
    print('\n## running {}'.format(label))
    if run_len is None: run_len = len(ds.enc_stamp) - start_idx # we go to the end
    odom_pos, odom_yaw = np.zeros((run_len, 3)), np.zeros((run_len, 1))
    pos0 = np.array(ds.truth_pos[start_idx])
    yaw0 = tf.transformations.euler_from_quaternion(ds.truth_ori[start_idx], axes='sxyz')[2]
    print('starting odom at {} {}'.format(pos0, yaw0))
    odom_pos[0], odom_yaw[0] = odom.start(pos0, yaw0, ds.enc_lw[start_idx], ds.enc_rw[start_idx], ds.enc_stamp[start_idx])
    for i in range(1, run_len):
        odom_pos[i], odom_yaw[i] = odom.update(ds.enc_lw[i+start_idx], ds.enc_rw[i+start_idx], ds.enc_dt[i+start_idx-1], i)
    return odom_pos, odom_yaw


def plot2d(odom, odom_pos, odom_yaw, ds, label):
    figsize=(20.48, 10.24)
    margins = 0.05, 0.07, 0.97, 0.95, 0.14, 0.39
    fig = hio.plot2d(ds)
    print('got {} odom'.format(len(odom_pos)))
    plt.plot(odom_pos[:,0], odom_pos[:,1], '.', label=label)
    #plt.gca().legend(True)

def plot_err(odom_pos, odom_yaw, ds, label, start_idx, run_len, filename):
    figsize=(20.48, 10.24)
    margins = 0.05, 0.07, 0.97, 0.95, 0.14, 0.39
    fig = jpu.prepare_fig(window_title='Odometry error {} ({})'.format(label, filename), figsize=figsize, margins=margins)

    if run_len is None: run_len = len(ds.enc_stamp) - start_idx # we go to the end
    truth_pos_1 = hio.interpolate(ds.truth_pos, ds.truth_stamp, ds.enc_stamp[start_idx:start_idx+run_len])
    pos_err = odom_pos - truth_pos_1
    ax = plt.subplot(2,1,1)
    plt.plot(ds.enc_stamp[start_idx:start_idx+run_len], np.linalg.norm(pos_err, axis=1))
    jpu. decorate(ax, title='odom translation error', xlab='time in s', ylab='m', legend=None, xlim=None, ylim=None)

    truth_yaw = np.array([tf.transformations.euler_from_quaternion(q, axes='sxyz')[2] for q in ds.truth_ori])
    truth_yaw_1 = hio.interpolate(truth_yaw[:,np.newaxis], ds.truth_stamp, ds.enc_stamp[start_idx:start_idx+run_len])
    yaw_err = hcu.wrap_angle(odom_yaw - truth_yaw_1)
    ax = plt.subplot(2,1,2)
    plt.plot(ds.enc_stamp[start_idx:start_idx+run_len], np.rad2deg(yaw_err))
    jpu. decorate(ax, title='odom rotation error', xlab='time in s', ylab='deg', legend=None, xlim=None, ylim=None)
      
if __name__ == '__main__':

    #ds = hio.DataSet('/home/poine/work/homere/homere_control/data/odom_gazebo_2.npz', _type='homere')
    #ds = hio.DataSet('/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_4.npz', _type='oscar')
    filename, _type = '/home/poine/work/julie/julie/julie_control/scripts/julie_odom_data_1.npz', 'homere'
    #filename, _type = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_3.npz', 'homere'
    ds = hio.DataSet(filename, _type)
    reg = fod.Regression(ds)
    reg.fit_wheel_radius()
    reg.fit_wheel_sep()

    #wheel_sep, wheel_radius = 0.11039085, 0.03078012
    wheel_sep, wheel_radius = reg.wheel_sep, reg.wheel_radius
    if True: # python implementation
        odom, label = Odometer(wheel_sep, wheel_radius, wheel_radius), 'pyodom'
        _start_idx, _run_len = 0, None
        odom_pos, odom_yaw = run_on_ds(odom, ds, label, _start_idx, _run_len)
        plot2d(odom, odom_pos, odom_yaw, ds, label)
        plot_err(odom_pos, odom_yaw, ds, label, _start_idx, _run_len, filename)
        #pdb.set_trace()
    if False: # C++ implementation
        odom2, label = cpp_homere_control.Odometry(), 'cppodom'
        odom2.init(wheel_sep, wheel_radius, wheel_radius)
        odom_pos2, odom_yaw2 = run_on_ds(odom2, ds, label)
        plot2d(odom2, odom_pos2, odom_yaw2, ds, label)
        plot_err(odom_pos2, odom_yaw2, ds, label)

    plt.show()
