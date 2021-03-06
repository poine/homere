#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, logging, numpy as np, matplotlib.pyplot as plt
import keras, pickle

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as hio, homere_control.utils as hcu
import lvel_io_utils as lvu
import pdb

class LvelIoAnn:
    delay = 3
    v_km1, v_km2, v_km3, u_km1, u_km2, u_km3, input_size = range(7) # lv_km1, lv_km2, lv_km3, pwm_s_km1, pwm_s_km2, pwm_s_km3
    
    def prepare_dataset(self, ds):
        wr, ws = 0.2, 0.75
        lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds.enc_vel_lw, ds.enc_vel_rw)])
        pwm_sum = ds.lw_pwm + ds.rw_pwm
        X = lr_vels_from_enc[:,0]
        U = pwm_sum[:-1]
        _ann_out = X[self.delay:]
        _ann_in = np.zeros((len(X)- self.delay, self.input_size))
        for i in range(self.delay, len(X)):
            _ann_in[i-self.delay, self.v_km1] = X[i-1]
            _ann_in[i-self.delay, self.v_km2] = X[i-2]
            _ann_in[i-self.delay, self.v_km3] = X[i-3]
            _ann_in[i-self.delay, self.u_km1] = U[i-1]
            _ann_in[i-self.delay, self.u_km2] = U[i-2]
            _ann_in[i-self.delay, self.u_km3] = U[i-3]
        return _ann_in, _ann_out
        
    
    def train(self, _ds, force_retrain=False, plant_ann_filename='/tmp/lvel_io3_plant_ann.h5', epochs=100):

        if force_retrain or not os.path.isfile(plant_ann_filename): 
        
            # Build the plant identification ANN
            plant_i = keras.layers.Input((6,), name ="plant_i") #v_km1, v_km2, v_km3, u_km1, u_km2, u_km3
            plant_l = keras.layers.Dense(1, activation='linear', kernel_initializer='uniform',
                                         #kernel_regularizer=keras.regularizers.l1(0.005),
                                         input_shape=(6,), use_bias=True, name="plant")
            plant_o = plant_l(plant_i)
            self.plant_ann = keras.models.Model(inputs=plant_i, outputs=plant_o)
            self.plant_ann.compile(loss='mean_squared_error', optimizer='adam')

            #
            _ann_in, _ann_out = self.prepare_dataset(_ds)
            
            self.plant_ann.fit(_ann_in, _ann_out, epochs=epochs, batch_size=32,  verbose=1, shuffle=True)
            # Save it to avoid retraining
            self.plant_ann.save(plant_ann_filename)
        else:
            # load a previously trained ANN
            self.plant_ann = keras.models.load_model(plant_ann_filename)

    def predict(self, vkm1, vkm2, vkm3, ukm1, ukm2, ukm3):
        return self.plant_ann.predict(np.array([[vkm1, vkm2, vkm3, ukm1, ukm2, ukm3]]))



    
def test_plant(plant_ann, ds):
    wr, ws = 0.2, 0.75
    lr_vels_from_enc = np.array([hcu.kin_vel_of_wheel_vel(lw_rvel, rw_rvel, wr, ws) for lw_rvel, rw_rvel in zip(ds.enc_vel_lw, ds.enc_vel_rw)])
    pwm_sum = ds.lw_pwm + ds.rw_pwm
    U = pwm_sum[:-1]
    Xpred = np.zeros((len(ds.enc_vel_stamp), 1))
    Xpred[0] = lr_vels_from_enc[0,0]
    Xpred[1] = lr_vels_from_enc[1,0]
    Xpred[2] = lr_vels_from_enc[2,0]
    for i in range(3, len(ds.enc_vel_stamp)):
        Xpred[i] = plant_ann.predict(Xpred[i-1], Xpred[i-2], Xpred[i-3], U[i-1], U[i-2], U[i-3])

    X = lr_vels_from_enc[:,0]
    lvu.plot_test(plt.subplot(2,1,1), plt.subplot(2,1,2), ds.enc_vel_stamp, X, U, Xpred)
    #hio.plot_truth_vel(ds)
    #plt.subplot(2,1,1)
    #plt.plot(ds.enc_vel_stamp, Xpred[:,0]) 

    
            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO) 
    _fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_11_random_lrvel.npz', 'homere'
    _ds_train = hio.DataSet(_fn, _t)

    plant_ann = LvelIoAnn()
    plant_ann.train(_ds_train, force_retrain=False, epochs=100)

    _fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_16_step_pwm_sum.npz', 'homere'
    #_fn, _t = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_17_sine_pwm_sum.npz', 'homere'
    _ds_test = hio.DataSet(_fn, _t)
    test_plant(plant_ann, _ds_test)
    plt.show()
