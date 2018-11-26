#!/bin/bash

export ROS_MASTER_URI=http://192.168.1.1:11311
export ROS_HOSTNAME=name
export ROS_IP=192.168.1.2

source /opt/ros/kinetic/setup.bash
source /home/username/catkin_ws/devel/setup.bash
exec "$@"