
import os, logging, numpy as np, matplotlib.pyplot as plt
import julie_misc.plot_utils as jpu

def plot_test(ax1_, ax2_, stamp, X, U, Xpred):
    plt.sca(ax1_)
    plt.plot(stamp, X, label='truth') 
    plt.plot(stamp, Xpred, label='predicted') 
    jpu. decorate(ax1_, title='Linear velocity', xlab='time in s', ylab='m/s', legend=True)
    plt.sca(ax2_)
    #plt.plot(stamp, U, label='sum pwm')
    plt.fill_between(stamp, U, label='sum pwm')
    jpu. decorate(ax2_, title='U', xlab='time in s', ylab='', legend=True)


