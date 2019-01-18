# distutils: language = c++
#
#
# PyMulticol is a python Multicol interface using cython 
#
# Author: Poine-2016/2017
#

# I should look at that https://github.com/longjie/ros_cython_example

import numpy as np
import tf.transformations

cimport numpy as np
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map
from libc.string cimport memcpy
from cython.operator cimport dereference as deref, preincrement as inc

##
# this is to remove the ‘int _import_array()’ defined but not used [-Wunused-function] warning
cdef void foo():
    np.import_array()
def bar():
    foo()

cdef extern from "homere_control/odometry.h" namespace "homere_controller":
    cdef cppclass c_Odometry "homere_controller::Odometry":
        c_Odometry()
        void init(double wheel_separation, double left_wheel_radius, double right_wheel_radius)
        void starting(const double time)
        bool update(double left_pos, double right_pos, const double time)
        double getHeading() const
        double getX() const
        double getY() const
        void reset(double x, double y, double psi, double left_pos, double right_pos)
        
        
cdef class Odometry:
    cdef c_Odometry *thisptr   
    cdef double z0
    
    def __cinit__(self):
        self.thisptr = new c_Odometry()
        
    def init(self, ws, lwr, rwr):
        self.thisptr.init(ws, lwr, rwr)
        
    def start(self, pos0, yaw0, lw_angle, rw_angle, t0):
        self.thisptr.reset(pos0[0], pos0[1], yaw0, lw_angle, rw_angle)
        self.thisptr.starting(t0)
        self.z0 = pos0[2]
        print(self.z0)
        #self.pos, self.yaw = pos0, yaw0
        return pos0, yaw0

    def update(self, lw_angle, rw_angle, dt):
        self.thisptr.update(lw_angle, rw_angle, dt)
        #self.pos = [self.thisptr.getX(), self.thisptr.getY(), 0]
        #self.yaw = [self.thisptr.getHeading()]
        #return self.pos, self.yaw 
        pos = [self.thisptr.getX(), self.thisptr.getY(), self.z0]
        yaw = [self.thisptr.getHeading()]
        return pos, yaw 
