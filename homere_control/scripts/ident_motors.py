#!/usr/bin/env python
import time, math, rospy, numpy as np

import homere_control.msg


class debugCollector:
    def __init__(self):
        rospy.Subscriber('/homere_controller/debug', homere_control.msg.homere_controller_debug, self.debug_callback)
        self._stamp = []
        self._lw_angle, self._rw_angle = [], []
        self._lw_rvel, self._rw_rvel = [], []
        self._lw_rvel_f, self._rw_rvel_f = [], []
        self._lw_pwm, self._rw_pwm = [],[]
        self._data_nb = 0
        self._msg_nb = 0
        
    def debug_callback(self, msg):
        self._lw_angle += msg.lw_angle[:msg.nb_data]
        self._rw_angle += msg.rw_angle[:msg.nb_data]
        self._lw_rvel += msg.lw_rvel[:msg.nb_data]
        self._rw_rvel += msg.rw_rvel[:msg.nb_data]
        self._lw_rvel_f += msg.lw_rvel_f[:msg.nb_data]
        self._rw_rvel_f += msg.rw_rvel_f[:msg.nb_data]
        self._lw_pwm += msg.lw_pwm[:msg.nb_data]
        self._rw_pwm += msg.rw_pwm[:msg.nb_data]
        self._stamp += [_s.to_sec() for _s in msg.stamp[:msg.nb_data]]
        self._msg_nb += 1

    def save(self, filename):
        print('saving to {}'.format(filename))
        np.savez(filename,
                 lw_angle = np.array(self._lw_angle),
                 rw_angle = np.array(self._rw_angle),
                 lw_rvel = np.array(self._lw_rvel),
                 rw_rvel = np.array(self._rw_rvel),
                 lw_rvel_f = np.array(self._lw_rvel_f),
                 rw_rvel_f = np.array(self._rw_rvel_f),
                 lw_pwm = np.array(self._lw_pwm),
                 rw_pwm = np.array(self._rw_pwm),
                 stamp = np.array(self._stamp)
        )

class Node:
    def __init__(self):
        rospy.init_node('ident_motors')

    
    def run(self):
        self._d = debugCollector()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print('recorded {} messages'.format(self._d._msg_nb))
            rate.sleep()


if __name__ == '__main__':
    node = Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        print('recorded {} odometry and {} mocap'.format(node._d._msg_nb))
    node._d.save('/tmp/motor_data_1')



