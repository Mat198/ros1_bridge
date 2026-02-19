#!/bin/bash

function ros_source_env() 
{
	if [ -f "$1" ]; then
		echo "sourcing   $1"
		source "$1"
	else
		echo "notfound   $1"
	fi	
}

ros_source_env /ros1_msgs_ws/install/setup.bash # ROS 1 source
ros_source_env /opt/ros/humble/setup.bash
ros_source_env /ros2_msgs_ws/install/setup.bash
ros_source_env /bridge_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge $1
