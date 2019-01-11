#!/usr/bin/env python
import time, math, rospy, numpy as np, sys
import matplotlib.pyplot as plt
import scipy.signal

'''
 This is an attempt at identifying open loop static response of the wheel+motor system
'''


import pdb

import homere_control.msg
import julie_misc.plot_utils as jpu

class debugCollector:
    def __init__(self):
        rospy.Subscriber('/homere_controller/debug', homere_control.msg.homere_controller_debug, self.debug_callback)
        self._stamp = []
        self._lw_angle, self._rw_angle = [], []
        self._lw_rvel, self._rw_rvel = [], []
        self._lw_rvel_f, self._rw_rvel_f = [], []
        self._lw_pwm, self._rw_pwm = [],[]
        self._data_nb = 0
        self._msg_nb = 0
        
    def debug_callback(self, msg):
        self._lw_angle += msg.lw_angle[:msg.nb_data]
        self._rw_angle += msg.rw_angle[:msg.nb_data]
        self._lw_rvel += msg.lw_rvel[:msg.nb_data]
        self._rw_rvel += msg.rw_rvel[:msg.nb_data]
        self._lw_rvel_f += msg.lw_rvel_f[:msg.nb_data]
        self._rw_rvel_f += msg.rw_rvel_f[:msg.nb_data]
        self._lw_pwm += msg.lw_pwm[:msg.nb_data]
        self._rw_pwm += msg.rw_pwm[:msg.nb_data]
        self._stamp += [_s.to_sec() for _s in msg.stamp[:msg.nb_data]]
        self._msg_nb += 1

    def save(self, filename):
        print('saving to {}'.format(filename))
        np.savez(filename,
                 lw_angle = np.array(self._lw_angle),
                 rw_angle = np.array(self._rw_angle),
                 lw_rvel = np.array(self._lw_rvel),
                 rw_rvel = np.array(self._rw_rvel),
                 lw_rvel_f = np.array(self._lw_rvel_f),
                 rw_rvel_f = np.array(self._rw_rvel_f),
                 lw_pwm = np.array(self._lw_pwm),
                 rw_pwm = np.array(self._rw_pwm),
                 stamp = np.array(self._stamp)
        )

class Node:
    def __init__(self):
        rospy.init_node('ident_motors')

    
    def run(self):
        self._d = debugCollector()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print('recorded {} messages'.format(self._d._msg_nb))
            rate.sleep()

class DataSet:
    def __init__(self, filename):
        print('## loading {}'.format(filename))
        self.data =  np.load(filename)
        self._lw_angle, self._rw_angle = self.data['lw_angle'], self.data['rw_angle'] 
        #pdb.set_trace()

    def plot(self):
        ax = plt.subplot(2,2,1)
        plt.plot(self.data['stamp'], self.data['lw_angle'], label='lw_angle')
        plt.plot(self.data['stamp'], self.data['rw_angle'], label='rw_angle')
        jpu.decorate(ax, title="angles", xlab='time', ylab='rad', legend=True)
        ax = plt.subplot(2,2,2)
        plt.plot(self.data['stamp'], self.data['lw_rvel'], alpha=0.5, label='lw_rvel')
        plt.plot(self.data['stamp'], self.data['rw_rvel'], alpha=0.5, label='rw_rvel')
        plt.plot(self.data['stamp'], self.data['lw_rvel_f'], label='lw_rvel_f')
        plt.plot(self.data['stamp'], self.data['rw_rvel_f'], label='rw_rvel_f')
        jpu.decorate(ax, title="rvel", xlab='time', ylab='rad/s', legend=True)
        ax = plt.subplot(2,2,3)
        plt.plot(self.data['stamp'], self.data['lw_pwm'], label='lw_pwm')
        plt.plot(self.data['stamp'], self.data['rw_pwm'], label='rw_pwm')
        jpu.decorate(ax, title="pwm", xlab='time', ylab='-127,127', legend=True)


    def plot_time_distribution(self):
        self.enc_dt = self.data['stamp'][1:] - self.data['stamp'][:-1]
        plt.hist(self.enc_dt, bins=100)
        jpu.decorate(plt.gca(), title="timestamps", xlab='s', ylab='')


    def compute_derivatives(self):
        self.enc_vel_lw = self.data['lw_angle'][1:] - self.data['lw_angle'][:-1]
        #self.enc_vel_rw = self.enc_rw[1:] - self.enc_rw[:-1]
        self.enc_dt = self.data['stamp'][1:] - self.data['stamp'][:-1]
        self.enc_vel_lw /= self.enc_dt
        #self.enc_vel_rw /= self.enc_dt
        self.enc_vel_stamp = (self.data['stamp'][:-1] + self.data['stamp'][1:])/2
        
        fs, cutoff = 100., 2#3.2 # sampling freq and desired cutoff (Hz)
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        order = 2
        # 2.0*M_PI/TIME_CONSTANT with TIME_CONSTANT = 1
        b, a = scipy.signal.butter(order, normal_cutoff, btype='low', analog=False)
        #self.enc_vel_f_lw = scipy.signal.filtfilt(b, a, self.enc_vel_lw)
        self.enc_vel_f_lw = scipy.signal.lfilter(b, a, self.enc_vel_lw)
        
        ax = plt.subplot(2,1,1)
        plt.plot(self.data['stamp'], self.data['lw_angle'], label='lw_angle')
        jpu.decorate(ax, title="angles", xlab='time', ylab='rad', legend=True)
        ax = plt.subplot(2,1,2)
        plt.plot(self.data['stamp'], self.data['lw_rvel'], alpha=0.5, label='lw_rvel')
        plt.plot(self.data['stamp'], self.data['lw_rvel_f'], label='lw_rvel_f')
        plt.plot(self.enc_vel_stamp, self.enc_vel_lw, label='lw_rvel1')
        plt.plot(self.enc_vel_stamp, self.enc_vel_f_lw, label='lw_rvelf1')
        jpu.decorate(ax, title="rvel", xlab='time', ylab='rad/s', legend=True)
        



        
import scipy.interpolate

def ident(_ds):
    ax = plt.subplot(1,2,1)
    #plt.plot(_ds.data['lw_pwm'], _ds.data['lw_rvel_f'], '.')
    plt.scatter(_ds.data['lw_pwm'], _ds.data['lw_rvel_f'], s=0.1)
    jpu.decorate(ax, title="left wheel", xlab='lw_pwm', ylab='lw_rvel')#, legend=True)
    ax = plt.subplot(1,2,2)
    #plt.plot(_ds.data['rw_pwm'], _ds.data['rw_rvel_f'], '.')
    plt.scatter(_ds.data['rw_pwm'], _ds.data['rw_rvel_f'], s=0.1)
    jpu.decorate(ax, title="right wheel", xlab='rw_pwm', ylab='rw_rvel')#, legend=True)
    #ident_poly(_ds, order=1)
    #ident_spline(_ds)
    ident_ann(_ds)
 
def ident_poly(_ds, order):
    # polynomial interpolation
    rvel_test = np.arange(-4, 4, 0.01)
    lw_p = np.polyfit(_ds.data['lw_rvel_f'], _ds.data['lw_pwm'], order)
    print('{}'.format(lw_p))
    lw_pwm_test = np.polyval(lw_p, rvel_test)
    
    rw_p = np.polyfit(_ds.data['rw_rvel_f'], _ds.data['rw_pwm'], order)
    print('{}'.format(rw_p))
    rw_pwm_test = np.polyval(rw_p, rvel_test)
    
    ax = plt.subplot(1,2,1)
    plt.plot(lw_pwm_test, rvel_test)
    ax = plt.subplot(1,2,2)
    plt.plot(rw_pwm_test, rvel_test)

def ident_spline(_ds):
    # spline interpolation
    rvel_test = np.arange(-4, 4, 0.01)
    s = len(_ds.data['lw_rvel_f'])*3.5
    idx_sorted = np.argsort(_ds.data['lw_rvel_f'])
    x, y = _ds.data['lw_rvel_f'][idx_sorted], _ds.data['lw_pwm'][idx_sorted]
    #s = len(_ds.data['lw_rvel_f'][idx_sorted])*4
    lw_spl = scipy.interpolate.UnivariateSpline(x, y, s=s)
    print('lw spline {} knots'.format(len(lw_spl.get_knots())))
    ax = plt.subplot(1,2,1)
    plt.plot(lw_spl(rvel_test), rvel_test)

    idx_sorted = np.argsort(_ds.data['rw_rvel_f'])
    x, y = _ds.data['rw_rvel_f'][idx_sorted], _ds.data['rw_pwm'][idx_sorted]
    #s = len(_ds.data['rw_rvel_f'][idx_sorted])*4
    rw_spl = scipy.interpolate.UnivariateSpline(x, y, s=s)
    print('rw spline {} knots'.format(len(rw_spl.get_knots())))
    ax = plt.subplot(1,2,2)
    plt.plot(rw_spl(rvel_test), rvel_test)


import sklearn.neural_network
def relu(v): return v * (v > 0)

def ident_ann(_ds):
    params = {
    'hidden_layer_sizes': (3, ),    # 
    'activation': 'relu',            # 'identity', 'logistic', 'tanh', 'relu'
    'solver': 'lbfgs',               # 'lbfgs', 'sgd', 'adam'
    'verbose': False, 
    'random_state':1, 
    'max_iter':100000, 
    'tol':1e-30,
    }

    def print_ann(ann, name):
        #print('{} coefs\n{}\nintercept\n{}'.format(name, ann.coefs_, ann.intercepts_))
        print('## ANN {}'.format(name))
        for i, w in enumerate(ann.coefs_):
            print('W{}\n{}'.format(i+1, w))
        for i, b in enumerate(ann.intercepts_):
            print('B{}\n{}'.format(i+1, b))

    def write_cpp(anns, classes, filename="/tmp/feedforward.h"):
        with open(filename, 'w') as f:
            f.write('namespace homere_controller {\n')
            fmt = '  {}::{}() {{\n'
            for ann, classname in zip(anns, classes):
                f.write(fmt.format(classname, classname))
                for i, (w, b) in enumerate(zip(ann.coefs_, ann.intercepts_)):
                    #pdb.set_trace()
                    f.write('    W{}_ << {};\n'.format(i+1, np.array2string(w.ravel(), separator=',')[1:-1]))
                    f.write('    B{}_ << {};\n'.format(i+1, np.array2string(b.ravel(), separator=',')[1:-1]))
                f.write('  }\n')
            f.write('}\n')
            

            
    ann_lw = sklearn.neural_network.MLPRegressor(**params)
    X, Y = _ds.data['lw_rvel_f'].reshape((-1,1)), _ds.data['lw_pwm']
    ann_lw.fit(X, Y)
    print_ann(ann_lw, 'Left')
    ann_rw = sklearn.neural_network.MLPRegressor(**params)
    X, Y = _ds.data['rw_rvel_f'].reshape((-1,1)), _ds.data['rw_pwm']
    ann_rw.fit(X, Y)
    print_ann(ann_rw, 'Right')

    write_cpp((ann_lw, ann_rw), ('LeftFeedForward', 'RightFeedForward'))

    rvel_test = np.arange(-4, 4, 0.01)
    lw_pwm_pred = ann_lw.predict(rvel_test.reshape(-1, 1))
    ax = plt.subplot(1,2,1)
    plt.plot(lw_pwm_pred, rvel_test)
    rw_pwm_pred = ann_rw.predict(rvel_test.reshape(-1, 1))
    ax = plt.subplot(1,2,2)
    plt.plot(rw_pwm_pred, rvel_test)

    # test 
    rw_pwm_pred2 = np.zeros(len(rw_pwm_pred))
    Wi, W1 = ann_rw.coefs_
    Bi, B1 = ann_rw.intercepts_
    for i in range(len(rvel_test)):
        #pdb.set_trace()
        l1_i = relu(np.dot(rvel_test[i], Wi)+Bi)
        #l1_i = np.tanh(np.dot(rvel_test[i], Wi)+Bi)
        rw_pwm_pred2[i] = np.dot(l1_i ,W1)+B1
    plt.plot(rw_pwm_pred2, rvel_test)

    
if __name__ == '__main__':

    if 'record' in sys.argv:
        node = Node()
        try:
            node.run()
        except rospy.ROSInterruptException:
            print('recorded {} odometry and {} mocap'.format(node._d._msg_nb))
        node._d.save('/tmp/motor_data_1')
    elif 'plot' in sys.argv:
       _ds = DataSet('./motor_data_6.npz')
       #_ds.plot()
       plt.figure()
       ident(_ds)
       plt.show()


