#!/usr/bin/env python


import math, numpy as np
import matplotlib.pyplot as plt


#import homere_control.utils as hut
import pdb


def symsawtooth(t, n_stair=20, dt_stair=0.5, _min=-20, _max=20, t0=0):
    a = int(math.fmod(t-t0, dt_stair*(n_stair+1))/dt_stair)
    return _min + (_max - _min) * a / n_stair

class SymSawtooth:
    def __init__(self, n_stair=20, dt_stair=0.5, _min=-20, _max=20, t0=0):
        self.vals = np.concatenate((np.linspace(_min, _max, n_stair/2), np.linspace(_max, _min, n_stair/2)))
        self.t0 = t0
        self.dt_stair = dt_stair
        self.n_stair = n_stair
        self.duration = dt_stair*n_stair
        
    def get(self, t):
        #pdb.set_trace()
        step = int(math.fmod(t-self.t0, self.dt_stair*(self.n_stair))/self.dt_stair)
        return self.vals[step]

if __name__ == '__main__':
    time = np.arange(6, 18, 0.01)
    sst = SymSawtooth(t0=6)
    sp = np.array([sst.get(t) for t in time])

    plt.plot(time, sp)
    plt.show()
    #pdb.set_trace()