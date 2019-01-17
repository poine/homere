#!/usr/bin/env python
import time, math, rospy, numpy as np, tf2_ros
import geometry_msgs.msg

import pdb

import homere_control.utils as hcu, homere_control.ros_utils as hcru 

class Node:

    def __init__(self, dest=(0, 0, 0)):
        rospy.init_node('homere_controller_input_sender', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.ctl_in_msg = geometry_msgs.msg.Twist()
        self.ctl_input_pub = rospy.Publisher('/homere/homere_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.dest = np.array(dest)
        self.T_o2g = np.eye(4)
        self.T_o2g[:3,3] = dest
        self.psi_sp = -1.14
        

    def get_robot_state(self):
        robot_localized = False
        while not robot_localized:
            try:
                bl2o_transf = self.tf_buffer.lookup_transform(source_frame='base_link', target_frame='odom', time=rospy.Time(0))
                robot_localized = True
                #rospy.loginfo(" got robot odom location")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo_throttle(1., " waiting to get robot odometry pose")
        #print('world to bl {}'.format(odom_to_bl_transf))
        T_bl2o = hcru.T_of_tf_transf(bl2o_transf.transform)
        return T_bl2o
        
    def run_ctl(self):
        T_bl2o = self.get_robot_state()
        psi, direction, point = hcru.adp_of_T(T_bl2o)
        err_psi = hcu.wrap_angle(psi - self.psi_sp)
        Kp = 1
        self.rvel_sp = -Kp*err_psi 
        
        #pdb.set_trace()
        #T_bl2g = np.dot(self.T_o2g, T_bl2o)
        #t, q = hcru.tq_of_T(T_bl2g)
        #print(t)
        
    def send_command(self):
        self.ctl_in_msg.linear.x = 0.
        self.ctl_in_msg.angular.z = self.rvel_sp
        self.ctl_input_pub.publish(self.ctl_in_msg)
        
    def run(self):
         rate = rospy.Rate(50.)
         while not rospy.is_shutdown():
             now = rospy.Time.now()
             self.run_ctl()
             self.send_command()
             rate.sleep()


if __name__ == '__main__':
    try:
        Node().run()
    except rospy.ROSInterruptException:
        pass
