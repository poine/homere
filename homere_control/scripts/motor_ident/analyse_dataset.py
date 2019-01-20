#!/usr/bin/env python
import time, math, rospy, numpy as np, sys
import matplotlib.pyplot as plt

import ident_motors as idm


if __name__ == '__main__':
    _ds = idm.DataSet('/tmp/motor_data_6.npz')
    #_ds.plot()
    #_ds.plot_time_distribution()
    _ds.compute_derivatives()
    plt.show()
