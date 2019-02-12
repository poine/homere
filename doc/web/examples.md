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
	
	
## RosMip/Gazebo

	roslaunch rosmip_gazebo rosmip_world.launch gui:=false
	rosrun homere_control send_cmd_vel.py _signal_type:=random_2 cmd_topic:=/rosmip_balance_controller/cmd_vel
	rosrun rosmip_control record_debug_io.py ~/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_03.npz

# Summary

{% include odometry_examples.html %}


	
