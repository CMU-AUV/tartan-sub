#!/bin/bash

# Script that starts the submarine. Should be called by hand or on boot by
# systemd

# Remove old files if they exist
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
sudo rm /home/ubuntu/l4t_dfs.conf

# Add all necessary launch files here
roslaunch tartan-sub motion.launch &
sleep 5
roslaunch darknet_ros darknet_ros.launch &
sleep 5
roslaunch tartan-sub data_collection.launch &
sleep 5

wait
