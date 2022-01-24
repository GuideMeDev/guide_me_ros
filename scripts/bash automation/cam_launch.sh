#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash 

gnome-terminal -- roslaunch guide_me_ros rs-camera_imu_filter_madgwick.launch
