#!/usr/bin/env python
import time, math, rospy, numpy as np, sys
import matplotlib.pyplot as plt

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
        jpu.decorate(ax, title="angles", xlab='time', ylab='rad')#, legend=True)
        ax = plt.subplot(2,2,2)
        plt.plot(self.data['stamp'], self.data['lw_rvel'], label='lw_rvel')
        plt.plot(self.data['stamp'], self.data['rw_rvel'], label='rw_rvel')
        plt.plot(self.data['stamp'], self.data['lw_rvel_f'], label='lw_rvel_f')
        plt.plot(self.data['stamp'], self.data['rw_rvel_f'], label='rw_rvel_f')
        jpu.decorate(ax, title="rvel", xlab='time', ylab='rad/s')#, legend=True)
        ax = plt.subplot(2,2,3)
        plt.plot(self.data['stamp'], self.data['lw_pwm'], label='lw_pwm')
        plt.plot(self.data['stamp'], self.data['rw_pwm'], label='rw_pwm')
        jpu.decorate(ax, title="rpwm", xlab='time', ylab='-127,127')#, legend=True)

        

def ident(_ds):
    ax = plt.subplot(1,2,1)
    plt.plot(_ds.data['lw_pwm'], _ds.data['lw_rvel_f'], '.')
    jpu.decorate(ax, title="", xlab='lw_pwm', ylab='lw_rvel')#, legend=True)
    ax = plt.subplot(1,2,2)
    plt.plot(_ds.data['rw_pwm'], _ds.data['rw_rvel_f'], '.')
    jpu.decorate(ax, title="", xlab='rw_pwm', ylab='rw_rvel')#, legend=True)

        
if __name__ == '__main__':

    if 'record' in sys.argv:
        node = Node()
        try:
            node.run()
        except rospy.ROSInterruptException:
            print('recorded {} odometry and {} mocap'.format(node._d._msg_nb))
        node._d.save('/tmp/motor_data_1')
    elif 'plot' in sys.argv:
       _ds = DataSet('/tmp/motor_data_2.npz')
       #_ds.plot()
       ident(_ds)
       plt.show()


