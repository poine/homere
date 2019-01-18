import numpy as np, rospy, tf.transformations

import homere_control.utils as hcu


# TF messages
def list_of_position(p): return (p.x, p.y, p.z)
def list_of_orientation(q): return (q.x, q.y, q.z, q.w)

def t_q_of_transf_msg(transf_msg):
    return list_of_position(transf_msg.translation), list_of_orientation(transf_msg.rotation)


def T_of_nav_odom(_m):
    t = list_of_position(_m.pose.pose.position)
    q = list_of_orientation(_m.pose.pose.orientation)
    return hcu.T_of_t_q(t, q)

def T_of_tf_transf(_m):
    t, q = t_q_of_transf_msg(_m)
    return hcu.T_of_t_q(t, q)

def tf_transf_of_T(_m, T):
    t, q = hcu.tq_of_T(T)
    _m.translation.x, _m.translation.y, _m.translation.z = t
    _m.rotation.x, _m.rotation.y, _m.rotation.z, _m.rotation.w = q
    
