#!/usr/bin/env python


import sys, os, logging, math, numpy as np
import rospy, rospkg, sensor_msgs.msg, nav_msgs.msg, homere_control.msg
import julie_misc.utils as jmu

import pdb

'''
Records raw odometry and simulator truth to npz file
'''

class Node:
    def __init__(self):
        rospy.init_node('record_odometry')
        # encoders
        rospy.Subscriber('/homere/homere_controller/debug', homere_control.msg.homere_controller_debug, self.raw_odom_callback)
        self.left_wheel_angle = []
        self.right_wheel_angle = []
        self.odom_stamp = []
        self.raw_odom_nb = 0
        # truth
        rospy.Subscriber('/homere/base_link_truth', nav_msgs.msg.Odometry, self.gazebo_truth_callback)
        self.truth_pos = []
        self.truth_ori = []
        self.truth_lvel = []
        self.truth_rvel = []
        self.truth_stamp = []
        
        
    def raw_odom_callback(self, msg):
        self.left_wheel_angle += msg.lw_angle[:msg.nb_data]
        self.right_wheel_angle += msg.rw_angle[:msg.nb_data]
        self.odom_stamp += [_s.to_sec() for _s in msg.stamp[:msg.nb_data]]
        self.raw_odom_nb += 1

    def gazebo_truth_callback(self, msg):
        self.truth_pos.append( jmu.list_of_xyz(msg.pose.pose.position))
        self.truth_ori.append( jmu.list_of_xyzw(msg.pose.pose.orientation))
        self.truth_lvel.append( jmu.list_of_xyz(msg.twist.twist.linear))
        self.truth_rvel.append( jmu.list_of_xyz(msg.twist.twist.angular))
        self.truth_stamp.append(msg.header.stamp.to_sec())
        
    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print('recorded {} odometry and {} thruth'.format( len(self.odom_stamp), len(self.truth_stamp)))
            rate.sleep()

            
if __name__ == '__main__':
    np.set_printoptions(precision=6)
    node = Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        print('recorded {} odometry and {} thruth'.format( len(node.odom_stamp), len(node.truth_stamp)))
        
    output_filename = '/tmp/foo' if len(sys.argv) < 2 else sys.argv[1]
    print('saving to {}'.format(output_filename))
    np.savez(output_filename,
             encoders_lw = np.array(node.left_wheel_angle),
             encoders_rw = np.array(node.right_wheel_angle),
             encoders_stamp = np.array(node.odom_stamp),
             truth_pos   = np.array(node.truth_pos),
             truth_ori   = np.array(node.truth_ori),
             truth_lvel  = np.array(node.truth_lvel),
             truth_rvel  = np.array(node.truth_rvel),
             truth_stamp = np.array(node.truth_stamp))
    
