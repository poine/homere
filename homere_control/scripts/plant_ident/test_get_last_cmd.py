#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os, logging, numpy as np, matplotlib.pyplot as plt, keras, pickle, scipy.signal, control
import pdb

import julie_misc.plot_utils as jpu, so_lti, so_lti_io_id_keras as pann, so_lti_io_mrc_keras as pmrc




def plot(time, y, u, ysp):
    plt.subplot(2,1,1)
    plt.plot(time, y)
    plt.plot(time, ysp)
    plt.subplot(2,1,2)
    plt.plot(time, u)
    


def main(omega_plant=1., xi_plant=0.5,):
    
    plant = so_lti.IoPlantModel(omega_plant, xi_plant)
    a0, a1, b0, b1 = plant.a0, plant.a1, plant.b0, plant.b1
    print(plant.tf_disc)
    
    if 0:
        time =  np.arange(0., 20.05, 0.01)
        ysp = scipy.signal.square(0.3*time)
        y0, u0 = np.random.uniform(low=-1., high=1., size=2), np.random.uniform(low=-1., high=1., size=1)
        y0[1] = y0[0]+np.random.uniform(low=-0.1, high=0.1, size=1)
        y, u = plant.sim_io(time, y0, u0, so_lti.IoCtlCst(ysp))
        plot(time, y, u, ysp)
        plt.show()
    
    horiz = 2

    # build network
    net_i = keras.layers.Input((horiz+2,), name ="net_i") #y_k, y_km1, ..., y_kmh, u_kmh
    w0 = 1/a1*np.array([[b0], [b1], [1.], [-a0]]) + np.random.uniform(low=-100., high=100., size=(4,1))
    pdb.set_trace()
    net_l = keras.layers.Dense(1, activation='linear',
                               #kernel_initializer='uniform',
                               kernel_initializer=keras.initializers.Constant(w0),
                               input_shape=(horiz+1,), use_bias=False, name="net_l")
    net_o = net_l(net_i)
    _ann = keras.models.Model(inputs=net_i, outputs=net_o)
    _opt = keras.optimizers.Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=False)
    #_opt = keras.optimizers.SGD(lr=0.01, decay=1e-6, momentum=0.9, nesterov=True)
    #_ann.compile(loss='mean_squared_error', optimizer=_opt)
    #_ann.compile(loss='mean_squared_logarithmic_error', optimizer=_opt)
    _ann.compile(loss='mean_absolute_error', optimizer=_opt)
    
    # prepare dataset
    if 1:
        ds_size = int(1e5)
        time = np.arange(0., (horiz+1)*plant.dt, plant.dt)
        _input, _output = np.zeros((ds_size, horiz+2), dtype=np.float64), np.zeros((ds_size, 1), dtype=np.float64)
        _us = np.zeros((ds_size, horiz))
        for i in range(ds_size):
            ysp = np.random.uniform(low=-1., high=1., size=horiz+1)
            y0, u0 = np.random.uniform(low=-1., high=1., size=2), np.random.uniform(low=-1., high=1., size=1)
            y0[1] = y0[0]+np.random.uniform(low=-0.01, high=0.01, size=1)
            y, u = plant.sim_io(time, y0, u0, so_lti.IoCtlCst(ysp))
            _input[i,:-1], _input[i,-1] = y, u[0]
            _output[i] = u[-2]
            _us[i] = u[:-1]
        np.savez('/tmp/ds.npz', _input=_input, _output=_output)
    else:
        _data =  np.load('/tmp/ds.npz')
        _input, _output = _data['_input'], _data['_output']
        ds_size = len(_output)

    # plot dataset
    if True:
        #
        #for i in range(ds_size):
        #    plt.plot(time, _input[i][:-1])

        for i in range(horiz+1):
            ax = plt.subplot(2,horiz+1,i+1)
            plt.hist(_input[:, i])
            jpu. decorate(ax, title='$y_{{{}}}$'.format(i))
        for i in range(horiz):
            ax = plt.subplot(2,horiz+1,i+1+horiz+1)
            plt.hist(_us[:, i])
            jpu. decorate(ax, title='$u_{{{}}}$'.format(i))
        plt.show()

    # train
    if True:
        callbacks = [keras.callbacks.EarlyStopping(monitor='val_loss', patience=10),
                     keras.callbacks.ModelCheckpoint(filepath='/tmp/foo.h5', monitor='val_loss', save_best_only=True)]
    
        history = _ann.fit(_input, _output, epochs=1000, batch_size=16, verbose=1,
                           shuffle=True, validation_split=0.2, callbacks=callbacks)

        plt.figure()
        plt.plot(history.history['loss'])
        plt.plot(history.history['val_loss'])
        jpu.decorate(plt.gca(), 'loss', xlab='epochs', legend=['training', 'validation'])
        print('  trained weights:\n{}'.format(net_l.get_weights()))   

    # force
    if False:
        print('  orig weight:\n{}'.format(net_l.get_weights()))
        a0, a1, b0, b1 = plant.a0, plant.a1, plant.b0, plant.b1
        net_l.set_weights([1/a1*np.array([[b0], [b1], [1.], [-a0]])])
        print('  new weight:\n{}'.format(net_l.get_weights()))

    # evaluate
    res = _ann.predict(_input) - _output
    loss = np.mean(np.square(res))
    print('loss {:.2e}'.format(loss))
    plt.figure()
    plt.hist(res)
    plt.show()
    loss = _ann.evaluate(_input, _output)
    print('loss {:.2e}'.format(loss))

        
if __name__ == "__main__":
    keras.backend.set_floatx('float64')
    logging.basicConfig(level=logging.INFO)
    main()
    plt.show()
