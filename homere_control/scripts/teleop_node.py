#!/usr/bin/env python
import time, math, rospy, numpy as np, sys

import homere_control.msg, sensor_msgs.msg

class Node:
    def __init__(self):
        rospy.init_node('teleop_node')
        self.ctl_input_pub = rospy.Publisher('/homere_controller/cmd', homere_control.msg.homere_controller_input, queue_size=1)
        self.ctl_in_msg = homere_control.msg.homere_controller_input()
        self.joy_sub = rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.joy_cb, queue_size=1)
        self.steering_input, self.driving_input = 0., 0. 

    def joy_cb(self, msg):
        self.steering_input = msg.axes[3]
        self.driving_input = msg.axes[0]
        #print(self.steering_input)
        
    def publish(self):
        if 0:
            self.ctl_in_msg.mode = 0 # PWM
            pwm_sum, pwm_dif = self.driving_input * -40, self.steering_input * 20
            self.ctl_in_msg.pwm_l = pwm_sum + pwm_dif
            self.ctl_in_msg.pwm_r = pwm_sum - pwm_dif
        if 1:
            self.ctl_in_msg.mode = 1 # wheel rvel
            rvel_sum, rvel_dif = self.driving_input * -2., self.steering_input * -2.
            self.ctl_in_msg.rvel_l = rvel_sum + rvel_dif
            self.ctl_in_msg.rvel_r = rvel_sum - rvel_dif
        self.ctl_input_pub.publish(self.ctl_in_msg)
        
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            #print('recorded {} messages'.format(self._d._msg_nb))
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    n = Node()
    n.run()
