#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np, matplotlib.pyplot as plt
import tf.transformations
import sklearn.neural_network

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as hio, homere_control.utils as hcu
import pdb

class LvelIoAnn:
    delay = 2
    v_km1, v_km2, u_km1, u_km2, input_size = range(5) # lv_km1, lv_km2, pwm_s_km1, pwm_s_km2
    def __init__(self):
        params = {
            'hidden_layer_sizes':(), # 
            'activation':'relu',     # ‘identity’, ‘logistic’, ‘tanh’, ‘relu’
            'solver': 'adam',        # ‘lbfgs’, ‘sgd’, ‘adam’
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
            _input[i-self.delay, self.v_km1] = X[i-1]
            _input[i-self.delay, self.v_km2] = X[i-2]
            _input[i-self.delay, self.u_km1] = U[i-1]
            _input[i-self.delay, self.u_km2] = U[i-2]
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

    def predict(self, vkm1, vkm2, ukm1, ukm2):
        _inp = [[vkm1, vkm2, ukm1, ukm2]]
        return self.ann.predict(_inp)


def test(ann, ds_test):
    wr, ws = 0.2, 0.75
    lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds_test.enc_vel_lw, ds_test.enc_vel_rw)])
    pwm_sum = _ds_test.lw_pwm + _ds_test.rw_pwm
    U = pwm_sum[:-1]
    Xpred = np.zeros((len(ds_test.enc_vel_stamp), 1))
    Xpred[0] = lr_vels_from_enc[0,0]
    Xpred[1] = lr_vels_from_enc[1,0]
    for i in range(2, len(ds_test.enc_vel_stamp)):
        Xpred[i] = ann.predict(Xpred[i-1], Xpred[i-2], U[i-1], U[i-2])
    hio.plot_truth_vel(ds_test)
    plt.subplot(2,1,1)
    plt.plot(ds_test.enc_vel_stamp, Xpred[:,0])
        
def run(ds_train, ds_test):

    wr, ws = 0.2, 0.75
    lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds_train.enc_vel_lw, ds_train.enc_vel_rw)])
    X = lr_vels_from_enc[:,0]
    pwm_sum = _ds_train.lw_pwm + _ds_train.rw_pwm
    U = pwm_sum[:-1]
    ann = LvelIoAnn()
    ann.fit(X, U)
    ann.summary()
    test(ann, ds_test)
    
    #pdb.set_trace()
    
if __name__ == '__main__':
    _fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_11_random_lrvel.npz', 'homere'
    _ds_train = hio.DataSet(_fn, _t)
    #hio.plot_all(_ds_train)
    _fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_16_step_pwm_sum.npz', 'homere'
    #_fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_13_random_pwm_sum.npz', 'homere'
    _ds_test = hio.DataSet(_fn, _t)
    run(_ds_train, _ds_test)
    plt.show()

