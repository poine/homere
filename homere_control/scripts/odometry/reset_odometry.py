#!/usr/bin/env python
import time, math, rospy, numpy as np, tf2_ros, geometry_msgs.msg, nav_msgs.msg
import pdb

#import homere_control.srv
import homere_control.ros_utils as hru, homere_control.utils as hcu

class Node:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_msg = geometry_msgs.msg.TransformStamped()
        self.tf_msg.header.frame_id = rospy.get_param('~ref_frame', "world")
        self.tf_msg.child_frame_id = "odom"
        self.robot_truth_topic = rospy.get_param('~robot_truth_topic', "/homere/base_link_truth")

    def fetch_thruth(self):
        #print('fetching base_link to _world truth')
        _msg = rospy.wait_for_message(self.robot_truth_topic, nav_msgs.msg.Odometry)
        #print('got {}'.format(_msg))
        T_bl2w = hru.T_of_nav_odom(_msg)
        #print('got T_bl2w\n{}'.format(T_bl2w))
        return T_bl2w

    def fetch_odom(self):
        #print('fetching base_link to odom')
        robot_localized = False
        while not robot_localized:
            try:
                bl_2_odom_transf = self.tf_buffer.lookup_transform(target_frame='odom', source_frame='base_link', time=rospy.Time(0))
                robot_localized = True
                #rospy.loginfo(" got robot odom location")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo_throttle(1., " waiting to get robot world location")
        #print('world to bl {}'.format(odom_to_bl_transf))
        T_bl2o = hru.T_of_tf_transf(bl_2_odom_transf.transform)
        #print('got T_bl2o\n{}'.format(T_bl2o))
        return T_bl2o


    def compute_T_w2o(self):
        T_bl2w = self.fetch_thruth()
        T_bl2o = self.fetch_odom()
        T_w2o = np.dot(T_bl2o, np.linalg.inv(T_bl2w))
        return T_w2o

    def broadcast_w2o(self, T_o2w):
        self.tf_msg.header.stamp = rospy.Time.now()
        hru.tf_transf_of_T(self.tf_msg.transform, T_o2w)
        self.broadcaster.sendTransform(self.tf_msg)
        
    def compute_drift(self, T_o2w):
        T_w2o1 = self.compute_T_w2o()
        T_o2o1 = np.dot(T_w2o1, T_o2w)
        t, q = hcu.tq_of_T(T_o2o1)
        a,d,p = hcu.adp_of_T(T_o2o1)
        print('err angle {:.1f} deg dist {:.2f} m'.format(np.rad2deg(a), np.linalg.norm(t)))
        #pdb.set_trace()
        #print(np.linalg.norm(t))
        #print('\n{}\n'.format(T_o2o1))
    
    def run(self):
        T_w2o = self.compute_T_w2o()
        T_o2w = np.linalg.inv(T_w2o)
        print('broacasting world to odom\n{}'.format(T_w2o))
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.broadcast_w2o(T_o2w)
            self.compute_drift(T_o2w)
            rate.sleep()
    

# def call_reset():
#     x0 = world_to_bl_transf.transform.translation.x
#     y0 = world_to_bl_transf.transform.translation.y
#     rospy.wait_for_service('/homere/homere_controller/reset_odom')
#     #pdb.set_trace()
#     reset_odometry = rospy.ServiceProxy('/homere/homere_controller/reset_odom', homere_control.srv.srv_reset_odom)
#     try:
#         resp1 = reset_odometry(x0, y0, 0)
#     except rospy.ServiceException as exc:
#         print("Service did not process request: " + str(exc))

if __name__ == '__main__':
    try:
        rospy.init_node('homere_reset_odometry', anonymous=True)
        Node().run()
    except rospy.ROSInterruptException: pass
