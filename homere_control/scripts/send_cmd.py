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


class PwmRandom:
    def __init__(self, t0, zero_sum=False, zero_dif=False):
        self._min_dur, self._max_dur = 0.5, 5.
        self.zero_sum, self.zero_dif = zero_sum, zero_dif
        self.end_stage = t0
        #self.type_stage = 0

    def to_msg(self, t, msg):
        if t  >= self.end_stage:
            #self.type_stage = (self.type_stage+1)%2
            self.pwm_sum = 0 if self.zero_sum  else np.random.uniform(low=-30., high=30.)
            self.pwm_dif = 0 if self.zero_dif else np.random.uniform(low=-40., high=40.)
            self.end_stage += np.random.uniform(low=self._min_dur, high=self._max_dur)
        msg.pwm_l  = self.pwm_sum + self.pwm_dif
        msg.pwm_r  = self.pwm_sum - self.pwm_dif
        msg.mode = 0
        

class PwmStep:
    def __init__(self, a=15, dt=10, dt0=0):
        self.a = a
    def to_msg(self, t, msg):
        msg.mode = 0
        msg.pwm_l = step(t, -self.a, self.a)
        msg.pwm_r = msg.pwm_l
        
class PwmSine:
    def __init__(self, a=15, om=0.5, dt=None, dt0=0):
        self.a, self.om, self.dt0 = a, om, dt0
        if dt is not None: self.om = 2*np.pi/dt
         
    def to_msg(self, t, msg):
        msg.mode = 0
        msg.pwm_l = self.a*np.math.sin(self.om*(t+self.dt0))
        msg.pwm_r = msg.pwm_l
        
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

def run_native():
    rospy.init_node('homere_controller_input_sender', anonymous=True)
    ctl_input_pub = rospy.Publisher('/homere/homere_controller/cmd', homere_control.msg.homere_controller_input, queue_size=1)
    ctl_in_msg = homere_control.msg.homere_controller_input()
    now = rospy.Time.now().to_sec()
    #ctl_in = PwmRandom(now, zero_sum=True, zero_dif=False)
    #ctl_in = PwmStep(a=10, dt=5, dt0=1.25)
    ctl_in = PwmSine(a=40, dt=10, dt0=2.5)
    #ctl_in = PwmStairs()
    #ctl_in = PwmSymStairs()
    #ctl_in = RvelSine()
    #ctl_in = RvelStep()
    #ctl_in = RvelUDSawTooth()
    rate = rospy.Rate(50.)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        ctl_in.to_msg(now.to_sec(), ctl_in_msg)
        ctl_in_msg.stamp = now
        ctl_input_pub.publish(ctl_in_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        run_native()
    except rospy.ROSInterruptException: pass
