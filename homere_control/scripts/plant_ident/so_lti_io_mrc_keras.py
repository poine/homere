#! /usr/bin/env python
# -*- coding: utf-8 -*-

# https://www.jeremyjordan.me/batch-normalization/

import os, logging, numpy as np, matplotlib.pyplot as plt
import keras, pickle, scipy.signal
import control
import pdb

import julie_misc.plot_utils as jpu
import so_lti
import so_lti_io_id_keras as pann


#from tensorflow.python.keras import backend as K
#K.set_floatx('float64')

class SoLtiIoCtlFB:
    def __init__(self, ysp, yr, k=[0, 0, 0, 1, 0, 0]):
        self.ysp, self.yr, self.k = ysp, yr, np.asarray(k)
        
    def get(self, k, y_k, y_km1, u_km1):
        return np.dot(self.k, [self.yr[k+1], self.yr[k], self.yr[k-1], y_k, y_km1, u_km1])

#
# Analytical (closed form) version of control
#
class SoLtiIoMrcCF(SoLtiIoCtlFB):
    def __init__(self, ysp, yr, plant, omega_ref, xi_ref, omega_err, xi_err):
        err_track = so_lti.IoPlantModel(omega_err, xi_err)
        ref = so_lti.IoPlantModel(omega_ref, xi_ref)
        k0, k1, k2 = 1./plant.a1, err_track.b1/plant.a1, err_track.b0/plant.a1 
        k3 = (plant.b1 - err_track.b1)/plant.a1
        k4 = (plant.b0 - err_track.b0)/plant.a1
        k5 = -plant.a0 / plant.a1
        k = [k0, k1, k2, k3, k4, k5]
        self.ref = so_lti.IoPlantModel(omega_ref, xi_ref)
        SoLtiIoCtlFB.__init__(self, ysp, yr, k)

    def run(self, yr_kp1, yr_k, yr_km1, y_k, y_km1, u_km1):
        return np.dot(self.k, [yr_kp1, yr_k, yr_km1, y_k, y_km1, u_km1])

       
#
# Neural Network version of control
#
class SoLtiIoMrc:
    def __init__(self, plant_ann, omega_ref=3., xi_ref=1., omega_err=10., xi_err=0.7):
        self.omega_ref, self.xi_ref, self.omega_err, self.xi_err = omega_ref, xi_ref, omega_err, xi_err
        # Build the controller+plant network (for controller training)
        #  -controller network
        ref_input = keras.layers.Input((3,), name ="ctl_ref_i", dtype='float64')   # yr_kp1, yr_k, yr_km1
        state_input = keras.layers.Input((3,), name ="ctl_x_i", dtype='float64')   # y_k, y_km1, u_km1
        ctl_input = keras.layers.concatenate([ref_input, state_input])
        #w0 = np.array([2.00040019e+06,-3.72081327e+06,1.73906438e+06,-2.78785229e+05,2.60135930e+05,-9.99800020e-01])
        self.ctl_l = keras.layers.Dense(1, activation='linear',
                                        kernel_initializer='uniform',
                                        #kernel_initializer=keras.initializers.Constant(w0),
                                        #kernel_regularizer=keras.regularizers.l2(0.001),
                                        input_shape=(6,), use_bias=False, name="ctl")
        ctl_output = self.ctl_l(ctl_input)
        # -plant network
        plant_input =  keras.layers.concatenate([state_input, ctl_output]) # (y_k, y_km1, u_km1), (u_k)
        plant_l = plant_ann.get_layer(name="plant")
        plant_l.trainable = False
        plant_output = plant_l(plant_input)
        # -append controller and plant
        self.full_ann = keras.models.Model(inputs=[ref_input, state_input], outputs=plant_output)
        opt = keras.optimizers.Adam(lr=0.3, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=True)
        #opt = keras.optimizers.SGD(lr=0.01, decay=1e-6, momentum=0.9, nesterov=True)
        self.full_ann.compile(loss='mean_squared_error', optimizer=opt)
        print('full ann summary')
        self.full_ann.summary()
        
        # Build the controller-only network (for controller using)
        ctl_i = keras.layers.Input((6,), name ="ctl_i") # (yr_kp1, yr_k, yr_km1) (y_k, y_km1, u_km1)
        ctl_o = self.ctl_l(ctl_i)
        self.ann_ctl = keras.models.Model(inputs=ctl_i, outputs=ctl_o)
        print('ctl ann summary')
        self.ann_ctl.summary()


    def make_training_set(self, _nb):
        # simulate reference model trajectory
        # y_k, y_km1, u_k, u_km1
        ref_in = np.random.uniform(low=-1., high=1., size=(_nb, 4)).astype(np.float64)
        ref_out = [self.ref.io_dyn(y_k, y_km1, u_k, u_km1) for y_k, y_km1, u_k, u_km1 in ref_in]
        #ref_in, ref_out = np.zeros((_nb, 4), dtype=np.float32), np.zeros(_nb, dtype=np.float32)
        # simulate tracking error
        eps_in = np.random.uniform(low=-1., high=1., size=(_nb, 2)).astype(np.float64)
        eps_out = [self.track_err.io_dyn(y_k, y_km1, 0, 0) for y_k, y_km1 in eps_in]
        #eps_in, eps_out = np.zeros((_nb, 2), dtype=np.float32), np.zeros(_nb, dtype=np.float32)
        #pdb.set_trace()
        # format training data
        # (yr_kp1, yr_k, yr_km1) (y_k, y_km1, u_km1)
        _input = [np.zeros((_nb, 3), dtype=np.float64), np.zeros((_nb, 3), dtype=np.float64)] 
        # (y_kp1)
        _output = np.zeros((_nb, 1), dtype=np.float64)
        for k in range(_nb-1):
            _input[0][k] = [ref_out[k], ref_in[k,0], ref_in[k,1]]
            _input[1][k] = np.array([ref_in[k,0], ref_in[k,1], ref_in[k,3]]) + [eps_in[k,0], eps_in[k,1], 0] 
            _output[k] = ref_out[k] + eps_out[k]
        return _input, _output

    def train(self, filename, epochs=200, dt=0.01, _nb=int(10e3)):
        self.ref = so_lti.IoPlantModel(self.omega_ref, self.xi_ref)
        self.track_err = so_lti.IoPlantModel(self.omega_err, self.xi_err)
        _input, _output = self.make_training_set(_nb)

        callbacks = [keras.callbacks.EarlyStopping(monitor='val_loss', patience=10),
                     keras.callbacks.ModelCheckpoint(filepath='best_model.h5', monitor='val_loss', save_best_only=True)]
        self.history = self.full_ann.fit(_input, _output, epochs=epochs, batch_size=32, verbose=1,
                                         shuffle=True, validation_split=0.1, callbacks=callbacks)
        self.full_ann.save(filename)
        return _input, _output
        
    def load(self, filename):
        self.full_ann = keras.models.load_model(filename)
        self.ref = so_lti.IoPlantModel(self.omega_ref, self.xi_ref)
        self.track_err = so_lti.IoPlantModel(self.omega_err, self.xi_err)
    
    def run(self, yr_kp1, yr_k, yr_km1, y_k, y_km1, u_km1):
        return self.ann_ctl.predict(np.array([[yr_kp1, yr_k, yr_km1, y_k, y_km1, u_km1]]))
        

    def force_weight(self, plant):
        print('#\n#\n# Forcing ctl')
        w_orig = self.ctl_l.get_weights()
        k = debug_ctl(plant.omega, plant.xi, self.omega_ref, self.xi_ref, self.omega_err, self.xi_err, plot=False)
        w_orig[0][:,0]=k
        self.ctl_l.set_weights(w_orig)

    # for compat with analytic solution
    def get(self, k, y_k, y_km1, u_km1):
        return self.run(self.yr[k+1], self.yr[k], self.yr[k-1], y_k, y_km1, u_km1)
    
    
def plot_training(ann):
    _h = ann.history.history
    plt.figure()
    plt.plot(_h['loss'])
    plt.plot(_h['val_loss'])
    jpu.decorate(plt.gca(), 'loss', xlab='epochs', legend=['training', 'validation'])
    #plt.savefig('../../plots/ann/mrc_training_loss.png')

def report_ctl_id(plant, ctl):
    print('\n## report_ctl_id')
    print('\n## closed form controller')
    ctl_cf = SoLtiIoMrcCF(None, None, plant, ctl.omega_ref, ctl.xi_ref, ctl.omega_err, ctl.xi_err)
    k = ctl_cf.k[:,np.newaxis]
    print(k)
    
    w_identified = ctl.ctl_l.get_weights()[0]
    print('Identified controller: \n{}'.format(w_identified))

def plot_training_dataset(_ann_in, _ann_out):
    names =  'yr_kp1', 'yr_k', 'yr_km1', 'y_k', 'y_km1', 'u_km1'
    __ann_in = np.hstack(_ann_in)

    for i in range(6):
        ax = plt.subplot(1, 6, i+1)
        plt.hist(__ann_in[:,i])
        plt.title(names[i])



    
def plot_ctl(time, ysp, yr, y):
    plt.figure()
    ax = plt.subplot(2,1,1)
    plt.plot(time, ysp, label='sp')
    plt.plot(time, yr, label='ref')
    plt.plot(time, y, label='plant')
    jpu. decorate(ax, title='$y$', xlab='time in s', ylab='m', legend=True)
    ax = plt.subplot(2,1,2)
    plt.plot(time, y-yr, label='ref')
    jpu. decorate(ax, title='$\epsilon$', xlab='time in s', ylab='m', legend=True)


def debug_ctl(omega_plant, xi_plant, omega_ref, xi_ref, omega_err, xi_err, plot=False):
    time =  np.arange(0., 6., 0.01)
    plant = so_lti.IoPlantModel(omega_plant, xi_plant)
    ysp = scipy.signal.square(time)
    ref = so_lti.IoPlantModel(omega_ref, xi_ref)
    yr, ur = ref.sim_io(time, [0, 0], so_lti.IoCtlCst(ysp))

    if 0:
        err_track = so_lti.IoPlantModel(omega_err, xi_err)
        k0, k1, k2 = 1./plant.a1, err_track.b1/plant.a1, err_track.b0/plant.a1 
        k3 = (plant.b1 - err_track.b1)/plant.a1
        k4 = (plant.b0 - err_track.b0)/plant.a1
        k5 = -plant.a0 / plant.a1
        k = [k0, k1, k2, k3, k4, k5]
    else:
        ctl = SoLtiIoMrcCF(ysp, yr, plant, omega_ref, xi_ref, omega_err, xi_err)
        k = ctl.k
        y, u = plant.sim_io(time, [1, 1], ctl)
  
    if plot:
        plot_ctl(time, ysp, yr, y)
    return k
   
    
def test_ctl(plant, ctl):
    time =  np.arange(0., 8.05, plant.dt)
    ysp, yr = scipy.signal.square(time), np.zeros(len(time))
    for i in range(2,len(time)):
        yr[i] = ctl.ref.io_dyn(yr[i-1], yr[i-2], ysp[i-1], ysp[i-2])
    y, u = np.zeros(len(time)), np.zeros(len(time))
    y[0] = y[1] = 1.2
    for k in range(1,len(time)-1):
        u[k] = ctl.run(yr[k+1], yr[k], yr[k-1], y[k], y[k-1], u[k-1])
        y[k+1] = plant.io_dyn(y[k], y[k-1], u[k], u[k-1])
    plot_ctl(time, ysp, yr, y)


def debug2(ctl):
    #_input, _output = ctl.make_training_set(int(10e3))
    time =  np.arange(0., 80.05, 0.01)
    ysp, yr = scipy.signal.square(time), np.zeros(len(time))
    for k in range(1,len(time)-1):
        yr[k+1] = ctl.ref.io_dyn(yr[k], yr[k-1], ysp[k], ysp[k-1])
    _nb = len(time)-2
    # (yr_kp1, yr_k, yr_km1) (y_k, y_km1, u_km1)
    _input = [np.zeros((_nb, 3), dtype=np.float32), np.zeros((_nb, 3), dtype=np.float32)] 
    # (y_kp1)
    _output = np.zeros((_nb, 1), dtype=np.float32)
    for k in range(1, len(time)-1):
        _input[0][k-2] = [yr[k+1], yr[k], yr[k-1]]
        _input[1][k-2] = [yr[k], yr[k-1], ysp[k-1]]
        _output[k-2]   = yr[k+1]
    scores = ctl.full_ann.evaluate(_input, _output, batch_size=32, verbose=1)
    print(scores)
    pdb.set_trace()
        
def main(train_ctl=True, ctl_epochs=5000,
         force_ctl=True, _test_ctl=True):

    ## Real plant
    omega_plant, xi_plant= 0.1, 0.3
    plant = so_lti.IoPlantModel(omega_plant, xi_plant)
    ## Ann plant
    plant_ann = pann.main(train_plant=False, plant_epochs=120,
                          force_plant=False, _test_plant=False)
    ##
    ## Controller
    ##
    omega_ref, xi_ref, omega_err, xi_err = 3, 0.9, 10, 0.7
    ctl = SoLtiIoMrc(plant_ann.plant_ann, omega_ref, xi_ref, omega_err, xi_err)
    ctl_ann_filename='/tmp/full_ann.h5'
    if train_ctl or not os.path.isfile(ctl_ann_filename):
        _ann_in, _ann_out = ctl.train(ctl_ann_filename, epochs=ctl_epochs, dt=0.01, _nb=int(10e3))
        plot_training(ctl)
        plot_training_dataset(_ann_in, _ann_out)
    else:
        ctl.load(ctl_ann_filename)
    
    if force_ctl:
        ctl.force_weight(plant)
    report_ctl_id(plant, ctl)

    debug2(ctl)
    
    if 0:
        time = np.arange(0, 100, plant.dt)
        ysp = scipy.signal.square(time)
        yr, ur = plant.sim_io(time, [0, 0], so_lti.IoCtlCst(ysp))
        ctl.yr = yr
        y, u = plant.sim_io(time, [1, 1], ctl)
        plot_ctl(time, ysp, yr, y)
        plt.show()
        
    if _test_ctl:
        #time = np.arange(0, 100, plant.dt)
        #ysp = scipy.signal.square(time)
        #yr, ur = plant.sim_io(time, [0, 0], so_lti.IoCtlCst(ysp))
        #ctl = SoLtiIoMrcCF(ysp, yr, plant, omega_ref=3., xi_ref=0.9, omega_err=10., xi_err=0.7)
        test_ctl(plant, ctl)

    
    plt.show()


if __name__ == "__main__":
    keras.backend.set_floatx('float64')
    logging.basicConfig(level=logging.INFO)
    main()
    #debug_ctl(omega_plant=0.1, xi_plant=0.3, omega_ref=3, xi_ref=0.9, omega_err=10, xi_err=0.7, plot=True)
    plt.show()
