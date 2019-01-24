#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, logging, numpy as np, matplotlib.pyplot as plt
import keras, pickle

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as hio, homere_control.utils as hcu
import lvel_io_utils as lvu
import lvel_io_keras as lvp
import so_lti
import pdb



class RefModel:

    def __init__(self):
        pass


class LVelIOCtl:

    def __init__(self, plant_ann):
        # build the controller network
        ref_input = keras.layers.Input((4,), name ="ctl_ref_i")   # Xr1_kp1, Xr2_kp1, Xr1_k, Xr2_k         
        state_input = keras.layers.Input((2,), name ="ctl_x_i")   # X1_k, X2_k
        ctl_input = keras.layers.concatenate([ref_input, state_input])
        ctl_l = keras.layers.Dense(1, activation='linear', kernel_initializer='uniform',
                                   input_shape=(6,), use_bias=False, name="ctl")
        ctl_output = ctl_l(ctl_input)
        # build the plant network (reuse provided layers)
        plant_input =  keras.layers.concatenate([state_input, ctl_output]) # X1_k, X2_k, U_k
        plant_l = plant_ann.get_layer(name="plant")
        plant_l.trainable = False
        #plant_output = plant_l(plant_input)
        # build the (controller + plant) network
        #self.full_ann = keras.models.Model(inputs=[ref_input, state_input], outputs=plant_output)
        #self.full_ann.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])
        #self.full_ann.summary()


        
    def train(self, omega_ref=3., xi_ref=1., omega_err=10., xi_err=0.7):
        # build LTI systems for reference model and tracking error dynamics
        ref = so_lti.CCPlant(omega_ref, xi_ref)
        track_err = so_lti.CCPlant(omega_err, xi_err)
        # simulate and record reference model trajectory
        time, Xr, Ur, desc = so_lti.make_or_load_training_set(ref, so_lti.CtlNone(), True)
        so_lti.plot(time, Xr, Ur)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    plant_ann = lvp.LvelIoAnn()
    plant_ann.train(None, force_retrain=False)    
    ctl = LVelIOCtl(plant_ann.plant_ann)
    ctl.train()
    plt.show()
