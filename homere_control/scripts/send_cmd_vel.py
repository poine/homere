#!/usr/bin/env python
import time, math, rospy
from geometry_msgs.msg import Twist

import homere_control.msg

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

def _pwm_sine(now, msg, a=15, om=0.5):
    #msg.linear.x =  10*math.sin(0.5*now)
    #msg.linear.y =  10*math.cos(0.5*now)
    msg.mode = 0
    msg.pwm_l = a*math.sin(om*now)
    msg.pwm_r = a*math.sin(om*now)

def _pwm_step(now, msg, a0=-10, a1=10, dt=4, t0=0):
    msg.mode = 0
    v =  a0 if math.fmod(now+t0, dt) > dt/2 else a1
    msg.pwm_l = msg.pwm_r = v


def _rvel_step(now, msg):
    msg.mode = 1
    msg.rvel_l = msg.rvel_r = step(now, a0=.0, a1=0.9, dt=20, t0=0)
    msg.rvel_l = 0

def _rvel_sine(now, msg, a=1, om=0.5):
    msg.mode = 1
    msg.rvel_l = msg.rvel_r = a*math.sin(om*now)
    #msg.rvel_l = 0

    
    
def run():
    rospy.init_node('homere_controller_input_sender', anonymous=True)
    #velocity_publisher = rospy.Publisher('/homere_controller/cmd_vel', Twist, queue_size=10)
    ctl_input_pub = rospy.Publisher('/homere_controller/cmd', homere_control.msg.homere_controller_input, queue_size=1)
    #vel_msg = Twist()
    ctl_in_msg = homere_control.msg.homere_controller_input()
    rate = rospy.Rate(50.)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        #_pwm_step(now, ctl_in_msg)
        _pwm_sine(now, ctl_in_msg)
        #_rvel_step(now, ctl_in_msg)
        #_rvel_sine(now, ctl_in_msg)
        #velocity_publisher.publish(vel_msg)
        ctl_input_pub.publish(ctl_in_msg)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
