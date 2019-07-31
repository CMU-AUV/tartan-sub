#!/bin/bash

# Script that starts the submarine. Should be called by hand or on boot by
# systemd

set -x

# Remove old files if they exist
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Add all necessary launch files here
roslaunch tartan-sub motion.launch &
sleep 5

python ~/catkin_ws/src/tartan-sub/src/robosub2019/qualify_run.py &

wait
