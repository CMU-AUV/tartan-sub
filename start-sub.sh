#!/bin/bash

# Script that starts the submarine. Should be called by hand or on boot by
# systemd

set -x

# Remove old files if they exist
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
sudo rm /home/ubuntu/l4t_dfs.conf

# Maximize the jetson's performance
set -e
sudo /home/ubuntu/jetson_clocks.sh --store
trap "sudo /home/ubuntu/jetson_clocks.sh --restore" EXIT

# Add all necessary launch files here
roslaunch robosub motion.launch &
sleep 5
roslaunch darknet_ros darknet_ros.launch &
sleep 5
rostopic pub /arming std_msgs/Bool "data: true" &
sleep 5

rosbag record -o ~/auto_run /zed/rgb/image_rect_color /mavros/imu/data &

python ~/catkin_ws/src/tartan-sub/src/mission_execute/target_hit.py &

wait
