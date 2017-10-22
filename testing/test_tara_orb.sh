#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/corvus/catkin_ws/devel/setup.bash
source /home/corvus/serial/local/setup.bash # from setup_serial.sh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/ORB_SLAM2_ROS/orb_slam2_ros:~/ORB_SLAM2_ROS/orb_slam2_lib

roslaunch ~/ORB_SLAM2_ROS/testing/tara_orb.launch
