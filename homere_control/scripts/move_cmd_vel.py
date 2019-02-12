#!/usr/bin/env python
import time, math, rospy, numpy as np, tf2_ros
import geometry_msgs.msg, sensor_msgs.msg, visualization_msgs.msg, nav_msgs.msg

import pdb
import two_d_guidance as tdg
import homere_control.msg
import homere_control.utils as hcu, homere_control.ros_utils as hcru 


def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

class Publisher:
    def __init__(self):
        self.pub_goal =  rospy.Publisher('/homere/move_controller/goal', visualization_msgs.msg.Marker, queue_size=1)
        self.goal_marker_msg = visualization_msgs.msg.Marker()
        self.goal_marker_msg.header.frame_id = "odom"
        self.goal_marker_msg.type = visualization_msgs.msg.Marker.SPHERE
        self.goal_marker_msg.pose.position.z = 0
        self.goal_marker_msg.pose.orientation.x = 0;
        self.goal_marker_msg.pose.orientation.y = 0;
        self.goal_marker_msg.pose.orientation.z = 0;
        self.goal_marker_msg.pose.orientation.w = 1;
        self.goal_marker_msg.scale.x = .1
        self.goal_marker_msg.scale.y = .1
        self.goal_marker_msg.scale.z = .1
        self.goal_marker_msg.color.a = 1.0
        self.goal_marker_msg.color.r = 1.0
        self.goal_marker_msg.color.g = 0.0
        self.goal_marker_msg.color.b = 0.0
        self.pub_path = rospy.Publisher('/homere/move_controller/path', nav_msgs.msg.Path, queue_size=1)
         
    def publish_goal(self, now, goal_pos):
        self.goal_marker_msg.header.stamp = now
        self.goal_marker_msg.pose.position.x = goal_pos[0]
        self.goal_marker_msg.pose.position.y = goal_pos[1]
        self.pub_goal.publish(self.goal_marker_msg)
        
    def publish_path(self, now, _path):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id="odom"
        for l in _path.points:
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

        
class Node:

    def __init__(self):
        rospy.init_node('homere_controller_input_sender', anonymous=True)
        cmd_topic = rospy.get_param('~cmd_topic', '/homere/homere_controller/cmd_vel')
 
        self.debug_pub = Publisher()
        
        goalx = rospy.get_param('~goalx', 0.)
        goaly = rospy.get_param('~goaly', 0.)
        self.goal = np.array([goalx, goaly])
        rospy.loginfo(" Setting goal to {}".format(self.goal))
        self.quit_on_arrival = rospy.get_param('~quit_on_arrival', False)
        rospy.loginfo(" Will {}quit on arrival".format('' if self.quit_on_arrival else 'not '))
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.imu_sub = rospy.Subscriber('/homere/imu', sensor_msgs.msg.Imu, self.imu_cbk, queue_size=1)
        
        self.ctl_in_msg = geometry_msgs.msg.Twist()
        self.ctl_input_pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.ctl_debug_msg = homere_control.msg.debug_move_controller()
        self.ctl_debug_pub = rospy.Publisher('/homere/move_controller/debug', homere_control.msg.debug_move_controller, queue_size=1)

        self.T_o2g = np.eye(4)
        self.T_o2g[:2,3] = self.goal
        #self.psi_sp = -1.14
        self.psid = 0
        self.sum_err_psi = 0.
        self.path = None
        

    def imu_cbk(self, msg):
        self.psid = msg.angular_velocity.z
        
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
        self.T_bl2o = hcru.T_of_tf_transf(bl2o_transf.transform)
        return self.T_bl2o
        
    def run_ctl(self, now):
        
        #self.psi_sp = step(now.to_sec(), a0=np.deg2rad(-20), a1=np.deg2rad(20), dt=20)

        T_bl2o = self.get_robot_state()
        self.psi, direction, point = hcu.adp_of_T(T_bl2o)
        self.pos = T_bl2o[:2,3]

        if self.path is None:
            self.path = tdg.path_factory.make_line_path(p0=self.pos, p1=self.goal, n_pt=100)
            self.path.reset()
            self.psi_sp = self.path.headings[0]
            rospy.loginfo(' made line path from {} to {} (heading {:.1f} deg)'.format(self.pos, self.goal, np.rad2deg(self.psi_sp)))

        self.look_ahead = 1.    
        p1, p2, self.end_reached, ip1, ip2 = self.path.find_carrot_alt(self.pos, _d=self.look_ahead)
        #print ip2, p2
        if self.end_reached:
            self.p2 = self.path.points[-1]
            #print('end')
        else:
            self.p2 = self.path.points[-1]
            #self.p2 = p2
            p0p2_w = p2 - self.pos
            #cy, sy = math.cos(self.psi), math.sin(self.psi)
            #w2b = np.array([[cy, sy],[-sy, cy]])
            #p0p2_b = np.dot(w2b, p0p2_w)
            self.psi_sp = np.arctan2(p0p2_w[1], p0p2_w[0])
        
        err_psi = hcu.wrap_angle(self.psi - self.psi_sp)
        err_psi = np.clip(err_psi, -np.pi/4, np.pi/4)
        #pdb.set_trace()
        #err_psid = 0
        self.sum_err_psi += err_psi
        Ki, Kp, Kd = 0.01, 3., 3.
        self.rvel_sp = -(Ki*self.sum_err_psi + Kp*err_psi + Kd*self.psid)
        self.lvel_sp = 0.2 if not self.end_reached else 0.
        
        #pdb.set_trace()
        #T_bl2g = np.dot(self.T_o2g, T_bl2o)
        #t, q = hcru.tq_of_T(T_bl2g)
        #print(t)
        
    def publish_command(self):
        self.ctl_in_msg.linear.x = self.lvel_sp
        self.ctl_in_msg.angular.z = self.rvel_sp
        self.ctl_input_pub.publish(self.ctl_in_msg)

    def publish_debug(self, now):
        self.ctl_debug_msg.stamp = now
        self.ctl_debug_msg.psi_sp = self.psi_sp
        self.ctl_debug_msg.psi_meas = self.psi
        self.ctl_debug_msg.psid_meas = self.psid
        self.ctl_debug_pub.publish(self.ctl_debug_msg)
        
    def run(self):
         rate = rospy.Rate(50.)
         while not rospy.is_shutdown():
             now = rospy.Time.now()
             self.run_ctl(now)
             self.publish_command()
             self.publish_debug(now)
             self.debug_pub.publish_goal(now, self.p2)
             self.debug_pub.publish_path(now, self.path)
             if self.quit_on_arrival and self.end_reached: rospy.signal_shutdown("end of path reached")
             rate.sleep()


if __name__ == '__main__':
    try:
        Node().run()
    except rospy.ROSInterruptException:
        pass
