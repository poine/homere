#!/usr/bin/env python


import math, numpy as np
import matplotlib.pyplot as plt
import tf.transformations

import julie_misc.plot_utils as jpu
import pdb


#
#
#
def T_of_t_q(t, q):
    T = tf.transformations.quaternion_matrix(q)
    T[:3,3] = t
    return T

def tq_of_T(T):
    return T[:3, 3], tf.transformations.quaternion_from_matrix(T)

def adp_of_T(T):
    angle, direction, point = tf.transformations.rotation_from_matrix(T)
    return angle, direction, point

#
#
#
def kin_vel_of_wheel_vel(lw_rvel, rw_rvel, w_r, w_s):
    lvel = (rw_rvel + lw_rvel) * 0.5 * w_r
    rvel = (rw_rvel - lw_rvel) * w_r / w_s
    return lvel, rvel

#
#
#
class SymSawtooth:
    def __init__(self, n_stair=20, dt_stair=0.5, _min=-20, _max=20, t0=0):
        self.vals = np.concatenate((np.linspace(_min, _max, n_stair/2), np.linspace(_max, _min, n_stair/2)))
        self.t0 = t0
        self.dt_stair = dt_stair
        self.n_stair = n_stair
        self.duration = dt_stair*n_stair
        
    def get(self, t):
        #pdb.set_trace()
        step = int(math.fmod(t-self.t0, self.dt_stair*(self.n_stair))/self.dt_stair)
        return self.vals[step]


def wrap_angle(_a): return ( _a + np.pi) % (2 * np.pi ) - np.pi
    
if __name__ == '__main__':
    time = np.arange(6, 18, 0.01)
    sst = SymSawtooth(t0=6)
    sp = np.array([sst.get(t) for t in time])

    plt.plot(time, sp)
    plt.show()
    #pdb.set_trace()

#
#
#
def plot_training(ann):
    _h = ann.history.history
    plt.figure()
    plt.plot(_h['loss'])
    plt.plot(_h['val_loss'])
    jpu.decorate(plt.gca(), 'loss', xlab='epochs', legend=['training', 'validation'])
