#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np, matplotlib.pyplot as plt
import tf.transformations
import sklearn.neural_network

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as hio, homere_control.utils as hcu
import pdb

class SklearnAnn:
    delay = 2
    x1_km1, x1_km2, x2_km1, x2_km2, u_km1, u_km2, u2_km1, u2_km2, input_size = range(9)
    def __init__(self):
        params = {
            'hidden_layer_sizes':(3), # 
            'activation':'relu',         # ‘identity’, ‘logistic’, ‘tanh’, ‘relu’
            'solver': 'adam',            # ‘lbfgs’, ‘sgd’, ‘adam’
            'verbose': True, 
            'random_state':1, 
            'max_iter':500, 
            'tol':1e-16,
            'warm_start': False
        }
        self.ann = sklearn.neural_network.MLPRegressor(**params)

    def make_input(self, X, U):
        _input = np.zeros(( len(X)- self.delay, self.input_size))
        for i in range(self.delay, len(X)):
            _input[i-self.delay, self.x1_km1] = X[i-1, 0]
            _input[i-self.delay, self.x1_km2] = X[i-2, 0]
            _input[i-self.delay, self.x2_km1] = X[i-1, 1]
            _input[i-self.delay, self.x2_km2] = X[i-2, 1]
            _input[i-self.delay, self.u_km1] = U[i-1,0]
            _input[i-self.delay, self.u_km2] = U[i-2,0]
            _input[i-self.delay, self.u2_km1] = U[i-1,1]
            _input[i-self.delay, self.u2_km2] = U[i-2,1]
        return _input
    
    def fit(self, X, U):
        print('building training set')
        ann_input, ann_output = self.make_input(X, U), X[self.delay:]
        print('fiting set')
        self.ann.fit(ann_input , ann_output)
        print(' done')
        print('score: {:e}'.format(self.ann.score(ann_input , ann_output)))

    def summary(self):
        c, w = self.ann.coefs_, self.ann.intercepts_
        print('{} {}'.format(c, w))

    def predict(self, Xkm1, Xkm2, Ukm1, Ukm2):
        _inp = [[Xkm1[0], Xkm2[0], Xkm1[1], Xkm2[1], Ukm1[0], Ukm2[0], Ukm1[1], Ukm2[1]]]
        return self.ann.predict(_inp)
    



def plot_foo(ds, X, U):
    fig = jpu.prepare_fig(window_title='Foo')

    ax = plt.subplot(3,1,1)
    plt.plot(ds.truth_vel_stamp, ds.truth_lvel_body[:,0], '.', label='vx')
    plt.plot(ds.truth_vel_stamp, X[:,0], label='vx')
    jpu. decorate(ax, title='linear velocity', xlab='time in s', ylab='m/s', legend=True)

    ax = plt.subplot(3,1,2)
    plt.plot(ds.truth_vel_stamp, ds.truth_rvel[:,2], '.', label='r')
    plt.plot(ds.truth_vel_stamp, X[:,1], label='r')
    jpu. decorate(ax, title='angular velocity', xlab='time in s', ylab='rad/s', legend=True)

    ax = plt.subplot(3,1,3)
    plt.plot(_ds.enc_stamp, _ds.lw_pwm, '.', label='lw_pwm')
    plt.plot(_ds.enc_stamp, _ds.rw_pwm, '.', label='rw_pwm')
    plt.plot(_ds.enc_stamp, U[:,0])
    plt.plot(_ds.enc_stamp, U[:,1])
    jpu. decorate(ax, title='pwm', xlab='time in s', ylab='', legend=True)
    
def run(ds, ds2):
    wr, ws = 0.2, 0.75
    #wr, ws = 0.2350, 1.1248
    lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds.enc_vel_lw, ds.enc_vel_rw)])
    if 1:
        hio.plot_truth_vel(_ds)
        plt.subplot(2,1,1)
        plt.plot(ds.enc_vel_stamp, lr_vels_from_enc[:,0])
        plt.subplot(2,1,2)
        plt.plot(ds.enc_vel_stamp, lr_vels_from_enc[:,1])
    ann = SklearnAnn()
    #X = np.vstack((ds.truth_lvel_body[:,0], ds.truth_rvel[:,2])).T
    X = lr_vels_from_enc
    U = np.vstack((_ds.lw_pwm, _ds.rw_pwm)).T[:-1]
    #plot_foo(ds, X, U)
    ann.fit(X, U)
    ann.summary()
    test(ann, ds2)

def test(ann, ds):
    wr, ws = 0.2, 0.75
    lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds.enc_vel_lw, ds.enc_vel_rw)])
    U = np.vstack((ds.lw_pwm, ds.rw_pwm)).T[:-1]
    Xpred = np.zeros((len(ds.enc_vel_stamp), 2))
    Xpred[0] = lr_vels_from_enc[0]
    Xpred[1] = lr_vels_from_enc[1]
    for i in range(2, len(ds.enc_vel_stamp)):
        Xpred[i] = ann.predict(Xpred[i-1], Xpred[i-2], U[i-1], U[i-2])
    #pdb.set_trace()
    #plt.hist()
    hio.plot_truth_vel(ds)
    plt.subplot(2,1,1)
    plt.plot(ds.enc_vel_stamp, Xpred[:,0])
    plt.subplot(2,1,2)
    plt.plot(ds.enc_vel_stamp, Xpred[:,1])

if __name__ == '__main__':
    filename, _type = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_10.npz', 'homere'
    #filename, _type = '/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_3.npz', 'homere'
    _ds = hio.DataSet(filename, _type)
    _ds2 = hio.DataSet('/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_4.npz', 'homere')

    #hio.plot_encoders(_ds)
    #hio.plot2d(_ds)
    #hio.plot_truth_vel(_ds)
    run(_ds, _ds2)
    plt.show()
