#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/pyrealsense2
export ROS_HOSTNAME=ubuntu-jetson
export ROS_MASTER_URI=http://192.168.1.7:11311
roslaunch camera_process realsense_camera.launch
