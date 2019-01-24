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
class SoLtiIoAnn:
    delay = 2
    y_k, y_km1, u_km1, u_k, input_size = range(5)

    def prepare_io(self, plant):
        # Record a trajectory of the real plant with a random input
        time = np.arange(0, 100, plant.dt)
        ysp = np.random.uniform(low=-1., high=1., size=len(time))
        y, u = plant.sim_io(time, [0, 0], SoLtiIoCtlCst(ysp))
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
            _ann_in, _ann_out = self.prepare_io(plant)
        else:
            _ann_in, _ann_out = self.prepare_io2(plant, int(10e3))
        # Fit the network
        self.history = self.plant_ann.fit(_ann_in, _ann_out, epochs=epochs, batch_size=64,
                                          verbose=1, shuffle=True, validation_split=0.1)
        # Save it to avoid retraining
        self.plant_ann.save(plant_ann_filename)
        return _ann_in, _ann_out

    def load(self, filename):
        # Load a previously trained ANN
        self.plant_ann = keras.models.load_model(filename)
            
    def predict(self, y_k, y_km1, u_km1, u_k):
        return self.plant_ann.predict(np.array([[y_k, y_km1, u_km1, u_k]]))


    def set_weights(self, _w):
        pass
        


def plot_training_dataset(_ann_in, _ann_out):
    names = 'y_k', 'y_km1', 'u_km1', 'u_k'
    for i in range(4):
        ax = plt.subplot(1, 4, i+1)
        plt.hist(_ann_in[:,i])
        plt.title(names[i])
    
def plot_training(ann):
    _h = ann.history.history
    plt.figure()
    plt.plot(_h['loss'])
    plt.plot(_h['val_loss'])
    jpu.decorate(plt.gca(), 'loss', xlab='epochs', legend=['training', 'validation'])
    #plt.savefig('../../plots/ann/mrc_training_loss.png')

def report_plant_id(plant, ann):
    plant.analyse()
    print('\n## real plant\n{}'.format(np.array([[-plant.b1], [-plant.b0], [plant.a1], [plant.a0]])))
    w_identified = ann.plant_ann.get_layer(name="plant").get_weights()[0]
    print('Identified plant: \n{}'.format(w_identified))
    num = np.array([w_identified[3,0], w_identified[2,0]], dtype=float)
    den = np.array([1, -w_identified[0,0], -w_identified[1,0]], dtype=float)
    dt = 0.01
    _tf = control.tf(num, den, dt)
    print('## identified plant')
    print(_tf)

def force_ann_plant(plant, ann):
    print('#\n#\n# Forcing plant')
    w_identified = ann.plant_ann.get_layer(name="plant").get_weights()
    w_identified[0][0,0] = -plant.b1
    w_identified[0][1,0] = -plant.b0
    w_identified[0][2,0] =  plant.a0
    w_identified[0][3,0] =  plant.a1
    ann.plant_ann.get_layer(name="plant").set_weights(w_identified)

def test_plant(plant, ann_plant):
    time =  np.arange(0., 25.05, plant.dt)
    sp, yp, ya = scipy.signal.square(0.25*time), np.zeros(len(time)) , np.zeros(len(time))
    for k in range(1,len(time)-1):
        yp[k+1] = plant.io_dyn(yp[k], yp[k-1], sp[k], sp[k-1])
        ya[k+1] = ann_plant.predict(ya[k], ya[k-1], sp[k-1], sp[k])

    ax = plt.gca()
    plt.plot(time, sp, label='sp')
    plt.plot(time, yp, label='plant')
    plt.plot(time, ya, label='ann')
    jpu. decorate(ax, title='$y$', xlab='time in s', ylab='m', legend=True)


def main(train_plant=False, plant_epochs=120,
         force_plant=False, _test_plant=False):

    plant = so_lti.IoPlantModel(omega=1., xi=0.3)

    ##
    ## Identify Plant
    ##
    plant_ann = SoLtiIoAnn()
    plant_ann_filename = '/home/poine/work/homere/homere_control/data/so_lti_io_plant_ann.h5'
    #plant_ann_filename = '/tmp/plant_ann.h5'
    if train_plant:
        _ann_in, _ann_out = plant_ann.train(plant, plant_epochs, plant_ann_filename)
        plot_training(plant_ann)
        report_plant_id(plant, plant_ann)
        plot_training_dataset(_ann_in, _ann_out)
        plt.show()
    else:
        plant_ann.load(plant_ann_filename)

    if force_plant:
        force_ann_plant(plant, plant_ann)
        report_plant_id(plant, plant_ann)

    if _test_plant:
        test_plant(plant, plant_ann)
        plt.show()

    return plant_ann

        
if __name__ == "__main__":
    keras.backend.set_floatx('float64')
    logging.basicConfig(level=logging.INFO)
    main(train_plant=True, plant_epochs=120, force_plant=False, _test_plant=True)
    plt.show()
