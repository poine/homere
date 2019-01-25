---
title: Odometry Examples
layout: default
---


# Obtaining dataset

## Julie/Gazebo

	roslaunch julie_simulations pure_pursuit_in_gazebo.launch world_name:=worlds/empty.world path_name:=test/oval_1.npz gui:=false
	rosrun julie_control record_odometry.py _output_filename:=/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_2.npz

	roslaunch julie_simulations pure_pursuit_in_gazebo.launch world_name:=worlds/empty.world path_name:=enac_outdoor_south_east/path_J_1.npz gui:=false
	rosrun julie_control record_odometry.py _output_filename:=/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_4.npz

## Homere/Gazebo

	roslaunch homere_simulator empty_world.launch gui:=false

	rosrun homere_control send_cmd_vel.py _signal_type:=step_ang _duration:=30
	rosrun homere_control move_cmd_vel.py _goalx:=10 _goaly:=10 _quit_on_arrival:=false

	
	./record_debug_io.py /home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_2.npz
	
	
	
	rosrun homere_control reset_odometry.py
	/home/poine/work/oscar.git/oscar/oscar_control/scripts/pp_guidance_node.py _twist_cmd_topic:=/homere/homere_controller/cmd_vel  _path_filename:=/home/poine/work/julie/julie/julie_worlds/paths/test/oval_2.npz
	rviz -d /home/poine/work/homere/homere_description/rviz/view_pp_guidance.rviz

# Summary

{% include odometry_examples.html %}


	
