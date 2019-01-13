#!/usr/bin/env python
import time, math, rospy, numpy as np
import geometry_msgs.msg
#from geometry_msgs.msg import Twist

import homere_control.msg
import homere_control.utils as hut
import pdb

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def stairs(t,  n_stair=10, dt_stair=0.5, _min=0, _max=20):
    a = int(math.fmod(t, dt_stair*n_stair)/dt_stair)
    return _min + (_max - _min) * a / n_stair

class PwmSine:
    def __init__(self, a=15, om=0.5, dt=None, dt0=0):
        self.a, self.om, self.dt0 = a, om, dt0
        if dt is not None: self.om = 2*np.pi/dt
        
    def to_msg(self, t, msg):
        msg.mode = 0
        msg.pwm_l = self.a*np.math.sin(self.om*(t+self.dt0))
        msg.pwm_r = -msg.pwm_l
        
class PwmStairs:
    def __init__(self, n_stair=10, dt_stair=2., _min=0, _max=20):
        self.n_stair, self.dt_stair, self._min, self._max = n_stair, dt_stair, _min, _max
    
    def to_msg(self, t, msg):
        msg.mode = 0
        msg.pwm_l = msg.pwm_r = stairs(t,  self.n_stair, self.dt_stair, self._min, self._max)

class PwmSymStairs:
    def __init__(self, n_stair=60, dt_stair=4., _min=-30, _max=30):
        self.sst = hut.SymSawtooth(n_stair, dt_stair, _min, _max, t0=0)
    def to_msg(self, t, msg):
        msg.mode = 0
        msg.pwm_l = msg.pwm_r = self.sst.get(t)

class RvelSine:
    def __init__(self, a=0.75, om=0.25):
        self.a, self.om = a, om
    def to_msg(self, t, msg):
        msg.mode = 1
        msg.rvel_l = msg.rvel_r = self.a*math.sin(self.om*t)

class RvelStep:
    def __init__(self, a0=0.5, a1=1., dt=10., t0=0):
        self.a0, self.a1, self.dt, self.t0 = a0, a1, dt, t0

    def to_msg(self, t, msg):
        msg.mode = 1
        msg.rvel_l = msg.rvel_r = step(t, self.a0, self.a1, self.dt, self.t0)

class RvelUDSawTooth:
    def __init__(self, n_stair=60, dt_stair=4., _min=-1.5, _max=1.5):
        self.sst = hut.SymSawtooth(n_stair, dt_stair, _min, _max, t0=0)
     
    def to_msg(self, t, msg):
        msg.mode = 1
        msg.rvel_l = msg.rvel_r = self.sst.get(t)

class TwistLinSine:
    def __init__(self, a=0.75, om=0.25):
        self.a, self.om = a, om
    def to_msg(self, t, msg):
        msg.linear.x = self.a*math.sin(self.om*t)
        msg.angular.z = 0
class TwistAngSine:
    def __init__(self, a=0.75, om=0.25):
        self.a, self.om = a, om
    def to_msg(self, t, msg):
        #pdb.set_trace()
        msg.linear.x = 0
        msg.angular.z = self.a*math.sin(self.om*t)        

class TwistLinStep:
    def __init__(self, a0=-0.1, a1=0.1, dt=10., t0=0):
        self.a0, self.a1, self.dt, self.t0 = a0, a1, dt, t0

    def to_msg(self, t, msg):
        msg.linear.x = step(t, self.a0, self.a1, self.dt, self.t0)
        msg.angular.z = 0

class TwistAngStep:
    def __init__(self, a0=-1., a1=1., dt=10., t0=0):
        self.a0, self.a1, self.dt, self.t0 = a0, a1, dt, t0

    def to_msg(self, t, msg):
        msg.linear.x = 0
        msg.angular.z = step(t, self.a0, self.a1, self.dt, self.t0)

class TwistRandom:
    def __init__(self, t0):
        self._min_dur, self._max_dur = 1., 10.
        self.end_stage = t0
        self.type_stage = 0
        
    def to_msg(self, t, msg):
        if t  >= self.end_stage:
            self.type_stage = (self.type_stage+1)%2
            self.v = np.random.uniform(low=-1., high=1.) if self.type_stage else 0
            self.psid = 0 if self.type_stage else np.random.uniform(low=-1., high=1.)
            self.end_stage += np.random.uniform(low=self._min_dur, high=self._max_dur)
        msg.linear.x = self.v
        msg.angular.z = self.psid
    
def run_native():
    rospy.init_node('homere_controller_input_sender', anonymous=True)
    ctl_input_pub = rospy.Publisher('/homere/homere_controller/cmd', homere_control.msg.homere_controller_input, queue_size=1)
    ctl_in_msg = homere_control.msg.homere_controller_input()
    now = rospy.Time.now().to_sec()
    #ctl_in = PwmSine(a=15, dt=10, dt0=2.5)
    #ctl_in = PwmStairs()
    #ctl_in = PwmSymStairs()
    ctl_in = RvelSine()
    #ctl_in = RvelStep()
    #ctl_in = RvelUDSawTooth()
    rate = rospy.Rate(50.)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        ctl_in.to_msg(now.to_sec(), ctl_in_msg)
        ctl_in_msg.stamp = now
        ctl_input_pub.publish(ctl_in_msg)
        rate.sleep()

def run_twist():
    rospy.init_node('homere_controller_input_sender', anonymous=True)
    ctl_input_pub = rospy.Publisher('/homere/homere_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    ctl_in_msg = geometry_msgs.msg.Twist()
    #ctl_in = TwistLinStep(a0=-0.1, a1=0.1, dt=60.)
    #ctl_in = TwistAngStep( a0=-0.7, a1=0.7, dt=40.)
    ctl_in = TwistRandom(t0=rospy.Time.now().to_sec())
    #ctl_in = TwistLinSine()
    #ctl_in = TwistAngSine()
    rate = rospy.Rate(50.)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        ctl_in.to_msg(now.to_sec(), ctl_in_msg)
        ctl_input_pub.publish(ctl_in_msg)
        rate.sleep()

# rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world odom 10        

if __name__ == '__main__':
    try:
        #run_native()
        run_twist()
    except rospy.ROSInterruptException: pass
