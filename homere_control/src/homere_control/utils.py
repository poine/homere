#!/usr/bin/env python


import math, numpy as np
import matplotlib.pyplot as plt
import tf.transformations

import julie_misc.plot_utils as jpu
import pdb

''' input test vectors '''

def make_random_pulses(dt, size, min_nperiod=1, max_nperiod=10, min_intensity=-1, max_intensity=1.):
    ''' make a vector of pulses of randon duration and intensities '''
    npulses = size/max_nperiod*2
    durations = np.random.random_integers(low=min_nperiod, high=max_nperiod, size=npulses)
    intensities =  np.random.uniform(low=min_intensity, high=max_intensity, size=npulses)
    pulses = []
    for duration, intensitie in zip(durations, intensities):
        pulses += [intensitie for i in range(duration)]
    pulses = np.array(pulses)
    time = np.linspace(0, dt*len(pulses), len(pulses))
    return time, pulses

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1
def sine_sweep(t, omega=2, domega=0.5, domega1=0.5): return math.sin(omega*(1-domega1*math.sin(domega*t))*t)


def random_input_vec(time): return np.random.uniform(low=-1.0, high=1.0, size=len(time))
def step_input_vec(time, a0=-1, a1=1, dt=4, t0=0): return [step(t, a0, a1, dt, t0) for t in time]
def sine_input_vec(time): return np.sin(time)
def sawtooth_input_vec(time): return scipy.signal.sawtooth(time)
def sine_swipe_input_vec(time): return [sine_sweep(t) for t in time]
"""
Compute numerical jacobian 
"""
def num_jacobian(X, U, dyn):
    s_size = len(X)
    i_size = len(U)
    epsilonX = (0.1*np.ones(s_size)).tolist()
    dX = np.diag(epsilonX)
    A = np.zeros((s_size, s_size))
    for i in range(0, s_size):
        dx = dX[i,:]
        delta_f = dyn(X+dx/2, 0, U) - dyn(X-dx/2, 0, U)
        delta_f = delta_f / dx[i]
        A[:,i] = delta_f

    epsilonU = (0.1*np.ones(i_size)).tolist()
    dU = np.diag(epsilonU)
    B = np.zeros((s_size,i_size))
    for i in range(0, i_size):
        du = dU[i,:]
        delta_f = dyn(X, 0, U+du/2) - dyn(X, 0, U-du/2)
        delta_f = delta_f / du[i]
        B[:,i] = delta_f

    return A,B

def get_om_xi(lambda1):
    om = math.sqrt(lambda1.real**2+lambda1.imag**2)
    xi = math.cos(np.arctan2(lambda1.imag, -lambda1.real))
    return om, xi

def get_lambdas(om, xi):
    re, im = -om*xi, om*math.sqrt(1-xi**2)
    return [complex(re, im), complex(re, -im)]

def get_precommand(A, B, C, K):
    tmp1 = np.linalg.inv(A - np.dot(B, K))
    tmp2 = np.dot(np.dot(C, tmp1), B)
    nr, nc = tmp2.shape
    H = -np.linalg.inv(tmp2) if nr == nc else -np.linalg.pinv(tmp2)
    return H


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


def rpy_of_q(q): return tf.transformations.euler_from_quaternion(q)

#
# Differential drive kinematic equation
# lw_rvel, rw_rvel are left wheel and right wheel rotational velocities, in rad/s
# wr is wheel radius, ws is wheel separation
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
