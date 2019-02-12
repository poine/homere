#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, logging, numpy as np, matplotlib.pyplot as plt
import keras, pickle, scipy.signal
import control
import pdb

import julie_misc.plot_utils as jpu
import so_lti





#
# Neural network plant identification
#
# Second Order LTI IO version
#
class SoLtiIoAnn:
    delay = 2
    y_k, y_km1, u_km1, u_k, input_size = range(5)

    def prepare_io_time(self, plant):
        # Record a trajectory of the real plant with a random input
        time = np.arange(0, 100, plant.dt)
        ysp = np.random.uniform(low=-1., high=1., size=len(time))
        y, u = plant.sim_io(time, [0, 0], so_lti.IoCtlCst(ysp))
        _ann_out = y[self.delay:]
        _ann_in = np.zeros((len(y)- self.delay, self.input_size))
        for i in range(self.delay, len(y)):
            _ann_in[i-self.delay, self.y_k]   = y[i-1]
            _ann_in[i-self.delay, self.y_km1] = y[i-2]
            _ann_in[i-self.delay, self.u_km1] = u[i-2]
            _ann_in[i-self.delay, self.u_k]   = u[i-1]
        return _ann_in, _ann_out

    def prepare_io2(self, plant, _nb):
        plant_in = np.random.uniform(low=-1., high=1., size=(_nb, 4))
        plant_out = np.array([plant.io_dyn(y_k, y_km1, u_k, u_km1) for y_k, y_km1, u_k, u_km1 in plant_in])
        return plant_in, plant_out
    
    def train(self, plant, epochs, plant_ann_filename):
        # Build the plant identification ANN
        plant_i = keras.layers.Input((4,), name ="plant_i") #y_k, y_km1, u_km1, u_k
        plant_l = keras.layers.Dense(1, activation='linear', kernel_initializer='uniform',
                                         input_shape=(4,), use_bias=False, name="plant")
        plant_o = plant_l(plant_i)
        self.plant_ann = keras.models.Model(inputs=plant_i, outputs=plant_o)
        self.plant_ann.compile(loss='mean_squared_error', optimizer='adam')
        # build delayed time series
        if 0:
            _ann_in, _ann_out = self.prepare_io_time(plant)
        else:
            _ann_in, _ann_out = self.prepare_io2(plant, int(10e3))
        # Fit the network
        callbacks = [keras.callbacks.EarlyStopping(monitor='val_loss', patience=10),
                     keras.callbacks.ModelCheckpoint(filepath='so_lti_plant_id_best_model.h5', monitor='val_loss', save_best_only=True)]
        self.history = self.plant_ann.fit(_ann_in, _ann_out, epochs=epochs, batch_size=64, verbose=1,
                                          shuffle=True, validation_split=0.1, callbacks=callbacks)
        # Save it to avoid retraining
        self.plant_ann.save(plant_ann_filename)
        return _ann_in, _ann_out

    def load(self, filename):
        # Load a previously trained ANN
        print('plant loading {}'.format(filename))
        self.plant_ann = keras.models.load_model(filename)
            
    def predict(self, y_k, y_km1, u_km1, u_k):
        return self.plant_ann.predict(np.array([[y_k, y_km1, u_km1, u_k]]))

    def weights_from_real_plant(self, plant):
        return [np.array([[-plant.b1], [-plant.b0], [plant.a1], [plant.a0]], dtype=np.float64)]

    def get_tf(self, dt=0.01):
        _w = self.plant_ann.get_layer(name="plant").get_weights()[0]
        num = np.array([_w[2,0], _w[3,0]], dtype=float)
        den = np.array([1, -_w[0,0], -_w[1,0]], dtype=float)
        _tf = control.tf(num, den, dt)
        return _tf
    
    def set_weights(self, _w):
        pass
        


def plot_training_dataset(_ann_in, _ann_out):
    fig = jpu.prepare_fig(window_title='Training dataset')
    names = '$y_k$', '$y_{k-1}$', '$u_{k-1}$', '$u_k$', '$y_{k+1}$'
    for i in range(4):
        ax = plt.subplot(1, 5, i+1)
        plt.hist(_ann_in[:,i])
        jpu.decorate(ax, title=names[i])
    ax = plt.subplot(1, 5, 5)
    plt.hist(_ann_out)
    jpu.decorate(ax, title=names[4])
        
def plot_training(ann):
    fig = jpu.prepare_fig(window_title='Training history')
    _h = ann.history.history
    plt.plot(_h['loss'], label='loss')
    plt.plot(_h['val_loss'], label='val_loss')
    jpu.decorate(plt.gca(), 'loss', xlab='epochs', legend=True)

def report_plant_id(plant, ann):
    print('\n## Real plant (as weights)\n{}'.format(ann.weights_from_real_plant(plant)[0].ravel()))
    print('tf: {}'.format(plant.get_tf()))
    print('\n## Identified plant weights: \n{}'.format(ann.plant_ann.get_layer(name="plant").get_weights()[0].ravel()))
    print('tf: {}'.format(ann.get_tf()))

def force_ann_plant(plant, ann):
    print('#\n#\n# Forcing plant')
    w_identified = ann.plant_ann.get_layer(name="plant").get_weights()
    w_identified[0][0,0] = -plant.b1
    w_identified[0][1,0] = -plant.b0
    w_identified[0][2,0] =  plant.a0
    w_identified[0][3,0] =  plant.a1
    ann.plant_ann.get_layer(name="plant").set_weights(w_identified)

def simulate_plant_and_ann(plant, ann_plant):
    time =  np.arange(0., 25.05, plant.dt)
    sp, yp, ya = scipy.signal.square(0.25*time), np.zeros(len(time)) , np.zeros(len(time))
    for k in range(1,len(time)-1):
        yp[k+1] = plant.io_dyn(yp[k], yp[k-1], sp[k], sp[k-1])
        ya[k+1] = ann_plant.predict(ya[k], ya[k-1], sp[k-1], sp[k])

    fig = jpu.prepare_fig(window_title='Time simulation')
    ax = plt.gca()
    plt.plot(time, sp, label='sp')
    plt.plot(time, yp, label='plant')
    plt.plot(time, ya, label='ann')
    jpu. decorate(ax, title='$y$', xlab='time in s', ylab='m', legend=True)


def main(omega_plant=1., xi_plant=0.5,
         train_plant=False, force_plant=False, _test_plant=False,
         plant_ann_filename='/home/poine/work/homere/homere_control/data/so_lti_io_plant_ann.h5',
         verbose=False):

    plant = so_lti.IoPlantModel(omega_plant, xi_plant)

    ##
    ## Identify Plant
    ##
    plant_ann = SoLtiIoAnn()
    if train_plant:
        _ann_in, _ann_out = plant_ann.train(plant, 120, plant_ann_filename)
        plot_training_dataset(_ann_in, _ann_out)
        plot_training(plant_ann)
        report_plant_id(plant, plant_ann)
    else:
        plant_ann.load(plant_ann_filename)
        if verbose: report_plant_id(plant, plant_ann)

    if force_plant:
        force_ann_plant(plant, plant_ann)
        report_plant_id(plant, plant_ann)

    if _test_plant:
        simulate_plant_and_ann(plant, plant_ann)

    return plant, plant_ann

        
if __name__ == "__main__":
    keras.backend.set_floatx('float64')
    logging.basicConfig(level=logging.INFO)
    main(train_plant=True, force_plant=False, _test_plant=True, verbose=True)
    plt.show()
