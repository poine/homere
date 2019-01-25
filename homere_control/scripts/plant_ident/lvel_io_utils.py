
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


def plot_rvel_test(ax1_, ax2_, stamp, X, U, Xpred):
    plt.sca(ax1_)
    plt.plot(stamp, X, label='truth') 
    plt.plot(stamp, Xpred, label='predicted') 
    jpu. decorate(ax1_, title='Angular velocity', xlab='time in s', ylab='m/s', legend=True)
    plt.sca(ax2_)
    #plt.plot(stamp, U, label='sum pwm')
    plt.fill_between(stamp, U, label='dif pwm')
    jpu. decorate(ax2_, title='U', xlab='time in s', ylab='', legend=True)


def plot_lrvel_test(ax1_, ax2_, ax3_, stamp, X, U, Xpred):
    plt.sca(ax1_)
    plt.plot(stamp, X[:,0], label='truth') 
    plt.plot(stamp, Xpred[:,0], label='predicted') 
    jpu. decorate(ax1_, title='Linear velocity', xlab='time in s', ylab='m/s', legend=True)
    plt.sca(ax2_)
    plt.plot(stamp, X[:,1], label='truth') 
    plt.plot(stamp, Xpred[:,1], label='predicted') 
    jpu. decorate(ax2_, title='Angular velocity', xlab='time in s', ylab='rad/s', legend=True)
    plt.sca(ax3_)
    #plt.plot(stamp, U, label='sum pwm')
    plt.fill_between(stamp, U[:,0], label='sum pwm')
    plt.fill_between(stamp, U[:,1], label='diff pwm')
    jpu. decorate(ax3_, title='U', xlab='time in s', ylab='', legend=True)
