#!/bin/bash
export ROS_MASTER_URI=http://192.168.1.7:11311
export ROS_IP=192.168.1.17
export ROS_HOSTNAME=nano

source /opt/ros/melodic/setup.bash
source /home/jetson/catkin_ws/devel/setup.bash

roslaunch camera_process cameras.launch
