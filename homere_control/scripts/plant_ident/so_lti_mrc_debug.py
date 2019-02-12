#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os, logging, numpy as np, matplotlib.pyplot as plt, keras, pickle, scipy.signal, control
import pdb

import julie_misc.plot_utils as jpu, so_lti, so_lti_io_id_keras as pann, so_lti_io_mrc_keras as pmrc

# In order for the network to be purely feedforward, we can not have u_km1 in the controller input
# during training. 
# 
def compute_prev_control_from_output_history(y, a0, a1, b0, b1, debug=False):
    if len(y) < 2: return 0
    _u = np.zeros(len(y))
    _u[:2] = [0,0]
    for k in range(2,len(y)):
        _u[k-1] = 1./a1*(y[k]+b1*y[k-1]+b0*y[k-2] - a0*_u[k-2])
    return _u if debug else _u[-2]


def compute_prev_control_from_output_history_horiz(y, a0, a1, b0, b1, h=-1, debug=False):
    print len(y)
    if len(y) < 2: return 0
    if h>0 and len(y)>h:
        _y = np.array(y[-h:])
    else:
        _y = y
    _u = np.zeros(len(_y)-1)
    __u = np.zeros(len(y)-1)
    #_u[0] = 0
    for k in range(2,len(_y)):
        _u[k-1]  = 1./a1*(_y[k]+b1*_y[k-1]+b0*_y[k-2] - a0*_u[k-2])
    for k in range(2,len(y)):
        __u[k-1] = 1./a1*(y[k]+b1*y[k-1]+b0*y[k-2] - a0*__u[k-2])
    #if h>0 and len(y)>h:
    #    pdb.set_trace()
    return _u if debug else _u[-1]

def compute_prev_control_from_output_history3(y, a0, a1, b0, b1, h):
    # TODO
    if len(y) < 9: return 0
    k=-1
    _u = np.zeros(min(len(y), h))
    for j in range(len(_u),-1,-1):
        u[k-j] = 1./a1*(y[k-j+1] + b1*y[k-j] + b0*y[k-j-1] -a0*-u[k-j-1])
    pdb.set_trace()
    u_km8 = 0
    u_km7 = 1./a1*(y[k-6] + b1*y[k-7] + b0*y[k-8] -a0*u_km8)
    u_km6 = 1./a1*(y[k-5] + b1*y[k-6] + b0*y[k-7] -a0*u_km7)
    u_km5 = 1./a1*(y[k-4] + b1*y[k-5] + b0*y[k-6] -a0*u_km6)
    u_km4 = 1./a1*(y[k-3] + b1*y[k-4] + b0*y[k-5] -a0*u_km5)
    u_km3 = 1./a1*(y[k-2] + b1*y[k-3] + b0*y[k-4] -a0*u_km4)
    u_km2 = 1./a1*(y[k-1] + b1*y[k-2] + b0*y[k-3] -a0*u_km3)
    u_km1 = 1./a1*(y[k]   + b1*y[k-1] + b0*y[k-2] -a0*u_km2)
    return u_km1


def compute_prev_control_from_output_history_mat(y, a0, a1, b0, b1, h, debug=False):
    if len(y) < 3: return 0
    if h>0 and len(y)> h:
       y=y[-h:]
    Ay = np.zeros((len(y)-2, len(y)))
    Au = np.zeros((len(y)-2, len(y)-1))
    for i in range(Ay.shape[0]):
        Ay[i,i:i+3] = 1, b1, b0
        Au[i,i:i+2] = a1, a0
    Z = np.dot(Ay, y[::-1])
    if 0: # assume u_kmh=0
        Au=Au[:, :-1]
        invAu = np.linalg.pinv(Au)
        U = np.dot(invAu, Z)
        U = np.hstack((U, [0]))
    if 1: # assumes u_kmh=0 with weights
        W = np.exp(-np.arange(len(y)-2)/10.)
        Au=Au[:, :-1]
        Au[-1,-1] += a0
        #S = np.sqrt(np.diag(W))
        #invAu = np.dot(np.linalg.inv(np.dot(np.dot(Au.T, S), Au)), Au.T)
        #U = np.dot(invAu, Z)
        Wd = np.sqrt(np.diag(W))
        Auw = np.dot(Wd, Au)
        Bw = np.dot(Z, Wd)
        U = np.linalg.lstsq(Auw, Bw)[0]
        U = np.hstack((U, U[-1]))
        pdb.set_trace()
    if 0: # solve underdeterminated system
        invAu = np.linalg.pinv(Au)
        U = np.dot(invAu, Z)
    if 0: # solve underdeterminated system with decreasing weights
        W = np.exp(-np.arange(len(y)-2)/5.)
        #W = np.arange(len(y)-2)[::-1]
        if 1:
            Auw = Au * np.sqrt(W[:,np.newaxis])
            Bw = Z * np.sqrt(W)
        else:
            Wd = np.sqrt(np.diag(W))
            Auw = np.dot(Wd, Au)
            Bw = np.dot(Z, Wd)
        pdb.set_trace()
        U = np.linalg.lstsq(Auw, Bw)[0]
    #if len(y)==20: pdb.set_trace()
    return U if debug else U[0]

def foo(plant, plant_ann):
    time = np.arange(0., 3.2, 0.01)
    ysp = np.random.uniform(low=-1., high=1., size=len(time)) #time + 0.3
    #ysp = scipy.signal.square(time)
    #ysp[0] = 0 
    y0, u0 = [0, 0], ysp[0]
    y, u = plant.sim_io(time, y0, u0, so_lti.IoCtlCst(ysp))
    a0, a1, b0, b1 = plant.a0, plant.a1, plant.b0, plant.b1
    
    #_u = compute_prev_control_from_output_history_horiz(y, a0, a1, b0, b1, debug=True)
    _u = compute_prev_control_from_output_history_mat(y, a0, a1, b0, b1, h=-1, debug=True)
    
    
    ax = plt.subplot(3,1,1)
    plt.plot(time, y, '.-')
    jpu. decorate(ax, title='y', legend=True)

    ax = plt.subplot(3,1,2)
    plt.plot(time[:-1], u[:-1], '.-',  label='truth')
    plt.plot(time[0:-1], _u[::-1], '.-', label ='est')

    jpu. decorate(ax, title='u', legend=True)
    err_u = _u[::-1] - u[:-1] 
    ax = plt.subplot(3,1,3)
    jpu. decorate(ax, title='$\\tilde{u}$', legend=True)
    plt.plot(time[:-1], err_u, '.-')
    #pdb.set_trace()
    
    plt.show()
    

    
def test_compute_previous_control(plant, plant_ann):
    time =  np.arange(0., 1.05, 0.01)
    #ysp = scipy.signal.square(time)
    #ysp = np.sin(time+1)
    ysp = np.random.uniform(low=-1., high=1., size=len(time))
    y, u = plant.sim_io(time, [1, 1], ysp[0], so_lti.IoCtlCst(ysp))
    u[0] = u [1]

    # test getting u_km1 from y_kp1, y_k, y_km1...y0
    a0, a1, b0, b1 = plant.a0, plant.a1, plant.b0, plant.b1
    _u1, _u2 = np.zeros(len(time)), np.zeros(len(time))
    horiz = 11
    for k in range(1, len(time)-1):
        _u1[k-1] = compute_prev_control_from_output_history(y[:k+1], a0, a1, b0, b1)
        #_u2[k-1] = compute_prev_control_from_output_history_horiz(y[:k+1], a0, a1, b0, b1, h=horiz)#, u1=u[:k])
        _u2[k-1] = compute_prev_control_from_output_history_mat(y[:k+1], a0, a1, b0, b1, h=horiz)#, u1=u[:k])
    _u1[-1] = _u2[-1] = u[-1]

    #pdb.set_trace()
    ax = plt.gca()
    err_u1 = _u1 - u
    err_u2 = _u2 - u
    plt.hist(err_u1[1:-3], label='u1', alpha=0.5)
    plt.hist(err_u2[1:-3], label='u2', alpha=0.5)
    jpu. decorate(ax, title='hist', legend=True)
 
    plt.figure()
    ax = plt.subplot(3,1,1)
    plt.plot(time, y, label='plant')
    jpu. decorate(ax, title='$y$')
    ax = plt.subplot(3,1,2)
    plt.plot(time[:-2], u[:-2], label='$u real$')
    plt.plot(time[:-2], _u1[:-2], label='$\\tilde{u}1$')
    plt.plot(time[:-2], _u2[:-2], label='$\\tilde{u}2$')
    jpu. decorate(ax, title='$u$', legend=True)
    ax = plt.subplot(3,1,3)
    plt.plot(time[1:-3], err_u1[1:-3], label='u1')
    plt.plot(time[1:-3], err_u2[1:-3], label='u2')
    jpu. decorate(ax, title='$u-\\tilde{u}$', legend=True)
    

# check that full ann works even if we can not train it yet
#  this works...
def test_ctl_full_ann(ctl, time, ysp):
    yr, ur = ctl.ref.sim_io(time, ysp[:2], so_lti.IoCtlCst(ysp))
    y, u = np.zeros(len(time)), np.zeros(len(time))
    y[0:2] = [1.2, 1.2]
    _nb = len(time)
    _input = [np.zeros((_nb, 3), dtype=np.float64), np.zeros((_nb, 3), dtype=np.float64)]
    _output = np.zeros((_nb, 2), dtype=np.float64)
    for k in range(1,len(time)-1):
        _input[0][k] = np.array([[yr[k+1], yr[k], yr[k-1]]])
        _input[1][k] = np.array([[y[k], y[k-1], u[k-1]]])
        y[k+1], u[k] = _output[k] = ctl.full_ann_with_ctl_out.predict([_input[0][k].reshape((1,3)), _input[1][k].reshape((1,3))]) 
    pmrc.plot_ctl(time, ysp, yr, y)
    return [_input[0][1:], _input[1][1:]], _output[1:,0][:, np.newaxis]


def def_full_ctl_ann_on_square(ctl, time=np.arange(0., 15.05, 0.01)):
    return test_ctl_full_ann(ctl, time, scipy.signal.square(0.3*time))
def def_full_ctl_ann_on_sine(ctl, time=np.arange(0., 15.05, 0.01)):
    return test_ctl_full_ann(ctl, time, np.sin(0.5*time))


# compute score of the full ann, that is the loss
# i cannot figure the results yet...
def score_full_ann(ctl, _input, _output):
    loss = ctl.full_ann.evaluate(_input, _output, batch_size=32, verbose=0)
    print('loss {:.2e}'.format(loss))
    pred_out = ctl.full_ann.predict(_input)
    pred_err = pred_out - _output
    loss = np.mean(np.square(pred_err))
    print('loss {:.2e}'.format(loss))



def make_good_dataset(ctl, plant, plant_ann):
    time =  np.arange(0., 15.05, 0.01)
    yrsp = np.random.uniform(low=-1., high=1., size=len(time))
    yr = np.zeros(len(time))
    yr[:2] = [0,0] # yrsp[:2] or what???
    for k in range(1,len(time)-1):
        yr[k+1] = plant_ann.predict(yr[k], yr[k-1], yrsp[k-1], yrsp[k])
    eps = np.zeros(len(time))
    _nb = len(time)
    _input = [np.zeros((_nb, 3), dtype=np.float64), np.zeros((_nb, 3), dtype=np.float64)]
    _output = np.zeros((_nb, 1), dtype=np.float64)
    a0, a1, b0, b1 = plant.a0, plant.a1, plant.b0, plant.b1
    for k in range(_nb-1):
        _input[0][k] = yr[k+1], yr[k], yr[k-1]
        _input[1][k] = yr[k], yr[k-1], compute_prev_control_from_output_history(yr[:k], a0, a1, b0, b1)
        _output[k] = yr[k+1]
    return _input, _output
        

def test_training_dataset(ctl, plant, plant_ann):
    _input, _output = make_good_dataset(ctl, plant, plant_ann)
    score_full_ann(ctl, _input, _output)
    

    
def main():
    ## Real and Ann plants
    plant, plant_ann = pann.main(train_plant=False, force_plant=False, _test_plant=False)
    ctl = pmrc.main(plant, plant_ann, train_ctl=False, force_ctl=True, _test_ctl=False)
    
    foo(plant, plant_ann)
    #test_compute_previous_control(plant, plant_ann)

    if 0:
        # works
        _input, _output =  def_full_ctl_ann_on_sine(ctl)

        # wtf!!! random network scores better than forced one
        score_full_ann(ctl, _input, _output)

        test_training_dataset(ctl, plant, plant_ann)
 
    
    plt.show()
    

if __name__ == "__main__":
    keras.backend.set_floatx('float64')
    logging.basicConfig(level=logging.INFO)
    main()
