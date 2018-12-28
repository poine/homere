#!/usr/bin/env python
import platform, signal, sys
import math, numpy as np
import rospy, sensor_msgs
from sensor_msgs.msg import Joy

import sabertooth


class Ctl:
    def __init__(self):
        self.k_linear  = 64.
        self.k_angular = 20.
        self.max_cmd = 127.
        self.linear = 0.
        self.angular = 0.
        
    def get(self, t):
        m1 = self.k_linear * self.linear - self.k_angular * self.angular
        m2 = self.k_linear * self.linear + self.k_angular * self.angular
        m1 = int(np.clip(m1, -self.max_cmd, self.max_cmd))
        m2 = int(np.clip(m2, -self.max_cmd, self.max_cmd))
        #m1, m2 = 0, 0
        return m1, m2

    def set(self, linear, angular):
        self.linear, self.angular = linear, angular


    
class Homere:
    def __init__(self, rate=50.):
        self.mc = sabertooth.SaberTooth() 
        self.ctl = Ctl()
        self.ctl_timeout = 0.5
        hostname = platform.uname()[1]
        rospy.init_node('Homere_{}'.format(hostname))
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.joy_callback)
        self.rate = rospy.Rate(rate)

    def run(self):
        print 'entering mainloop'
        start = rospy.get_rostime()
        self.last_input = start - rospy.Duration.from_sec(self.ctl_timeout)
        last = start - self.rate.sleep_dur
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            dt = now - last; last = now
            # if (now - start).to_sec() < 4.:
            if (now-self.last_input).to_sec() < self.ctl_timeout:
                cmd = self.ctl.get((now - start).to_sec())
                #print('no_timeout')
            else:
                cmd = 0,0
                #print 'timeout'
            #print cmd
            self.mc.send(*cmd)
            self.rate.sleep()

    def joy_callback(self, msg):
        self.last_input = rospy.get_rostime()
        linear, angular = msg.axes[3], msg.axes[0] 
        #if math.fabs(angular) < 0.04:
        #    angular = 0.
        self.ctl.set(linear, angular)
        
        
            
    def quit(self):
        print('Homere quiting')
        self.mc.quit()
        
        
if __name__ == '__main__':
    np.set_printoptions(precision=7, suppress=True, linewidth=200)
    homere = None
    def signal_handler(signal, frame):
        homere.quit()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    homere = Homere()
    try:
        homere.run()
    except rospy.ROSInterruptException:
        print "exiting"
