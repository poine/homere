#!/usr/bin/env python
import time, math, rospy, numpy as np
from geometry_msgs.msg import Twist

import homere_control.msg
import homere_control.utils as hut
import pdb

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def stairs(t,  n_stair=10, dt_stair=0.5, _min=0, _max=20):
    a = int(math.fmod(t, dt_stair*n_stair)/dt_stair)
    return _min + (_max - _min) * a / n_stair

class PwmSine:
    def __init__(self, a=15, om=0.5):
        self.a, self.om = a, om
    def to_msg(self, t, msg):
        msg.mode = 0
        msg.pwm_l = msg.pwm_r = self.a*np.math.sin(self.om*t)
    
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
    def __init__(self):
        pass
    def to_msg(self, t, msg, a=0.75, om=0.25):
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
        
def run():
    rospy.init_node('homere_controller_input_sender', anonymous=True)
    ctl_input_pub = rospy.Publisher('/homere_controller/cmd', homere_control.msg.homere_controller_input, queue_size=1)
    ctl_in_msg = homere_control.msg.homere_controller_input()
    now = rospy.Time.now().to_sec()
    ctl_in = PwmSine()
    #ctl_in = PwmStairs()
    #ctl_in = PwmSymStairs()
    #ctl_in = RvelSine()
    #ctl_in = RvelStep()
    #ctl_in = RvelUDSawTooth()
    rate = rospy.Rate(50.)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        ctl_in.to_msg(now, ctl_in_msg)
        ctl_input_pub.publish(ctl_in_msg)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
