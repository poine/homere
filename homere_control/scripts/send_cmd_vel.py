#!/usr/bin/env python
import time, math, numpy as np
import rospy, geometry_msgs.msg

import homere_control.msg, homere_control.utils as hcu
import pdb

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def stairs(t,  n_stair=10, dt_stair=0.5, _min=0, _max=20):
    a = int(math.fmod(t, dt_stair*n_stair)/dt_stair)
    return _min + (_max - _min) * a / n_stair

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

class TwistSine:
    def __init__(self, a1=0.75, om1=0.25, a2=0.75, om2=0.2):
        self.a1, self.om1 = a1, om1
        self.a2, self.om2 = a2, om2
    def to_msg(self, t, msg):
        msg.linear.x = self.a1*math.sin(self.om1*t)
        msg.angular.z = self.a2*math.sin(self.om2*t)
        
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
    def __init__(self, t0, zero_v=False, zero_psid=False, alternate=False):
        self._min_dur, self._max_dur = 1., 10.
        self.zero_v, self.zero_psid, self.alternate = zero_v, zero_psid, alternate
        self.end_stage = t0
        self.type_stage = 0
        
    def to_msg(self, t, msg):
        if t  >= self.end_stage:
            self.type_stage = (self.type_stage+1)%2
            self.v    = 0 if self.zero_v    or self.alternate and self.type_stage     else np.random.uniform(low=-1., high=1.)
            self.psid = 0 if self.zero_psid or self.alternate and not self.type_stage else np.random.uniform(low=-1., high=1.)
            self.end_stage += np.random.uniform(low=self._min_dur, high=self._max_dur)
        msg.linear.x = self.v
        msg.angular.z = self.psid
    
def run_twist():
    rospy.init_node('homere_controller_input_sender', anonymous=True)
    ctl_input_pub = rospy.Publisher('/homere/homere_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    ctl_in_msg = geometry_msgs.msg.Twist()

    duration = rospy.get_param('~duration', 60)
    signal_type = rospy.get_param('~signal_type', "step_lin")
    print('Sending {} for {:.1f} s'.format(signal_type, duration))
    ctl_ins = {'step_lin': lambda: TwistLinStep(a0=-0.1, a1=0.1, dt=60.),
               'step_ang': lambda: TwistAngStep( a0=-0.7, a1=0.7, dt=40.),
               'sine_lin': lambda: TwistLinSine(a=1., om=0.25),
               'sine_ang': lambda: TwistAngSine(a=1., om=0.25),
               'sine_2':   lambda: TwistSine(a1=1., om1=0.25, a2=1., om2=0.2),
               'random_lin': lambda: TwistRandom(t0=rospy.Time.now().to_sec(), zero_psid=True),
               'random_ang': lambda: TwistRandom(t0=rospy.Time.now().to_sec(), zero_v=True),
               'random_alt': lambda: TwistRandom(t0=rospy.Time.now().to_sec(), alternate=True),
               'random_2': lambda: TwistRandom(t0=rospy.Time.now().to_sec()) }
    ctl_in = ctl_ins[signal_type]()
    rate = rospy.Rate(50.)
    rospy.sleep(0.1)
    start = now = rospy.Time.now(); end = start + rospy.Duration(duration)
    #pdb.set_trace()
    i = 0
    while not rospy.is_shutdown() and now < end:
        now = rospy.Time.now()
        elapsed = now - start
        ctl_in.to_msg(now.to_sec(), ctl_in_msg)
        ctl_input_pub.publish(ctl_in_msg)
        if i%10 == 0: print('{:04.1f} s: lin: {:.2f} ang: {:.2f}'.format(elapsed.to_sec(), ctl_in_msg.linear.x, ctl_in_msg.angular.z))
        i += 1
        rate.sleep()

# rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world odom 10        

if __name__ == '__main__':
    try:
        run_twist()
    except rospy.ROSInterruptException: pass
