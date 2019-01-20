#!/usr/bin/env python
import os

cmd_go_home = "rosrun homere_control move_cmd_vel.py _goalx:=0 _goaly:=0 _quit_on_arrival:=true"
cmd_stay_home = "rosrun homere_control move_cmd_vel.py _goalx:=0 _goaly:=0 _quit_on_arrival:=false"

def make_sine():
    os.system(cmd_go_home)
    os.system("rosrun homere_control send_cmd_vel.py _signal_type:=sine_ang _duration:=30.")
    os.system(cmd_go_home)
    os.system("rosrun homere_control send_cmd_vel.py _signal_type:=sine_lin _duration:=30.")
    os.system(cmd_go_home)
    os.system("rosrun homere_control send_cmd_vel.py _signal_type:=sine_2 _duration:=30.")
    os.system(cmd_stay_home)

def make_steps():
    os.system(cmd_go_home)
    os.system("rosrun homere_control send_cmd_vel.py _signal_type:=step_ang _duration:=30.")
    os.system(cmd_go_home)
    os.system("rosrun homere_control send_cmd_vel.py _signal_type:=step_lin _duration:=30.")
    os.system(cmd_stay_home)

def make_random():
    #for _st, _sd in [('random_ang', 120), ('random_lin', 120), ('random_alt', 120), ('random_2', 240)]:
    for _st, _sd in [('random_lin', 240)]:
        os.system(cmd_go_home)
        os.system("rosrun homere_control send_cmd_vel.py _signal_type:={} _duration:={}".format(_st, _sd))
    os.system(cmd_stay_home)



if __name__ == '__main__':

    #make_steps()
    make_random()
