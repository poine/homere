#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, logging, numpy as np, matplotlib.pyplot as plt
import keras, pickle

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as hio, homere_control.utils as hcu
import lvel_io_utils as lvu

import pdb


'''
Plant identification with Keras
Linear and Rotational velocities using io presentation
'''

class LRvelIoAnn:
    delay = 2
    lv_km1, rv_km1, lv_km2, rv_km2, ps_km1, pd_km1, ps_km2, pd_km2, input_size = range(9) # rv_km1, rv_km2, pwm_d_km1, pwm_d_km2
    
    def prepare_dataset(self, ds):
        wr, ws = 0.2, 0.75
        lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds.enc_vel_lw, ds.enc_vel_rw)])
        pwm_sum, pwm_dif = ds.lw_pwm + ds.rw_pwm, ds.lw_pwm - ds.rw_pwm 
        X = lr_vels_from_enc
        U = np.vstack((pwm_sum, pwm_dif)).T[:-1]
        _ann_out = X[self.delay:]
        _ann_in = np.zeros((len(X)- self.delay, self.input_size))
        for i in range(self.delay, len(X)):
            _ann_in[i-self.delay, self.lv_km1] = X[i-1,0]
            _ann_in[i-self.delay, self.rv_km1] = X[i-1,1]
            _ann_in[i-self.delay, self.lv_km2] = X[i-2,0]
            _ann_in[i-self.delay, self.rv_km2] = X[i-2,1]
            _ann_in[i-self.delay, self.ps_km1] = U[i-1,0]
            _ann_in[i-self.delay, self.pd_km1] = U[i-1,1]
            _ann_in[i-self.delay, self.ps_km2] = U[i-2,0]
            _ann_in[i-self.delay, self.pd_km2] = U[i-2,1]
        return _ann_in, _ann_out
        
    
    def train(self, _ds, epochs, plant_ann_filename):

        # Build the plant identification ANN
        plant_i = keras.layers.Input((self.input_size,), name ="plant_i") 
        plant_l = keras.layers.Dense(2, activation='linear', kernel_initializer='uniform',
                                     input_shape=(self.input_size, ), use_bias=True, name="plant")
        plant_o = plant_l(plant_i)
        self.plant_ann = keras.models.Model(inputs=plant_i, outputs=plant_o)
        opt = keras.optimizers.Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=True)
        self.plant_ann.compile(loss='mean_squared_error', optimizer=opt)

        # build delayed time series
        _ann_in, _ann_out = self.prepare_dataset(_ds)
        # Fit the network
        callbacks = [keras.callbacks.EarlyStopping(monitor='val_loss', patience=10),
                     keras.callbacks.ModelCheckpoint(filepath='lvel_id_best_model.h5', monitor='val_loss', save_best_only=True)]
        self.history = self.plant_ann.fit(_ann_in, _ann_out, epochs=epochs, batch_size=64,  verbose=1,
                                          shuffle=True, validation_split=0.1, callbacks=callbacks)
        # Save it to avoid retraining
        self.plant_ann.save(plant_ann_filename)


    def load(self, plant_ann_filename):
        # load a previously trained ANN
        self.plant_ann = keras.models.load_model(plant_ann_filename)

    def predict(self, lvkm1, rvkm1, lvkm2, rvkm2, pskm1, pdkm1, pskm2, pdkm2):
        return self.plant_ann.predict(np.array([[lvkm1, rvkm1, lvkm2, rvkm2, pskm1, pdkm1, pskm2, pdkm2]]))



def test_plant(plant_ann, ds):
    wr, ws = 0.2, 0.75
    lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds.enc_vel_lw, ds.enc_vel_rw)])
    pwm_sum, pwm_dif = ds.lw_pwm + ds.rw_pwm, ds.lw_pwm - ds.rw_pwm 
    U =  np.vstack((pwm_sum, pwm_dif)).T[:-1]
    Xpred = np.zeros((len(ds.enc_vel_stamp), 2))
    Xpred[0] = lr_vels_from_enc[0]
    Xpred[1] = lr_vels_from_enc[1]
    for i in range(2, len(ds.enc_vel_stamp)):
        Xpred[i] = plant_ann.predict(Xpred[i-1,0], Xpred[i-1,1], Xpred[i-2, 0], Xpred[i-2, 1], U[i-1, 0], U[i-1, 1], U[i-2,0], U[i-2,1])
    X = lr_vels_from_enc
    return X, U, Xpred
    
def make_many_test(ann, _dir, _fns):
    _t = 'homere'
    fig = jpu.prepare_fig(window_title='Test Plant Prediction')
    nr, nc = 4,3
    for i, _fn in enumerate(_fns):
        ds = hio.DataSet(_dir+_fn, _t)
        X, U, Xpred = test_plant(ann, ds)
        lvu.plot_lrvel_test(plt.subplot(nr,nc,nc*i+2), plt.subplot(nr,nc,nc*i+3), plt.subplot(nr,nc,nc*i+1), ds.enc_vel_stamp, X, U, Xpred)
    
def main(train_plant=True): 
    plant_ann = LRvelIoAnn()
    plant_ann_filename='/home/poine/work/homere/homere_control/data/lrvel_io_plant_ann2.h5'
    if train_plant or not os.path.isfile(plant_ann_filename):
        #_fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_11_random_lrvel.npz', 'homere'
        _fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_10.npz', 'homere'
        _ds_train = hio.DataSet(_fn, _t)
        hio.plot_truth_vel(_ds_train)
        plant_ann.train(_ds_train, epochs=600, plant_ann_filename=plant_ann_filename)
        hcu.plot_training(plant_ann)
        plt.show()
    else:
        plant_ann.load(plant_ann_filename)

        
    if 0:
        _fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_15_sine_rvel_1.npz', 'homere'
        _ds_test = hio.DataSet(_fn, _t)
        X, U, Xpred = test_plant(plant_ann, _ds_test)
        lvu.plot_lrvel_test(plt.subplot(1,3,2), plt.subplot(1,3,3), plt.subplot(1,3,1), _ds_test.enc_vel_stamp, X, U, Xpred)
    else:
        if 1:
            make_many_test(plant_ann, '/home/poine/work/homere/homere_control/data/homere/gazebo/',
                           ['homere_io_16_step_pwm_10_sum.npz', 'homere_io_16_step_pwm_20_sum.npz',
                            'homere_io_16_step_pwm_30_sum.npz', 'homere_io_16_step_pwm_40_sum.npz'])
        if 1:
            make_many_test(plant_ann, '/home/poine/work/homere/homere_control/data/homere/gazebo/',
                           ['homere_io_17_sine_pwm_sum.npz', 'homere_io_17_sine_pwm_15_sum.npz',
                            'homere_io_17_sine_pwm_30_sum.npz', 'homere_io_17_sine_pwm_40_sum.npz'])
    plt.show()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()