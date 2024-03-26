#!/bin/bash

source /home/adminlab/catkin_ws/devel/setup.bash

ROS_SERVER_IP="192.168.200.2"
ROS_CLIENT_IP="192.168.200.2"
CUSTOM_ROS_URI="http://${ROS_SERVER_IP}:11311"
export ROS_MASTER_URI="${CUSTOM_ROS_URI}"
export ROS_HOSTNAME=${ROS_CLIENT_IP}
export ROS_IP=${ROS_CLIENT_IP}

roslaunch logitech_f710_joy_ros launch_wifibot_demo.launch video_device:=/dev/video0

