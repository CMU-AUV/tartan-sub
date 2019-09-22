cd ~/catkin_ws
source /usr/share/gazebo-9/setup.sh
source /opt/ros/melodic/setup.bash
source devel/setup.bash

source ~/.bashrc

rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
# rosdep install --from-paths src --ignore-src --rosdistro=indigo -y --skip-keys "gazebo gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control gazebo_ros_pkgs"

catkin_make install

cd src/tartan-sub/simulation-essentials/robosub-worlds
cp * ~/catkin_ws/src/uuv_simulator/uuv_gazebo_worlds/worlds/

cd ..
cd robosub-worlds-models
cp -r * ~/catkin_ws/src/uuv_simulator/uuv_gazebo_worlds/models/

cd ..
cp velocity_control.launch  ~/catkin_ws/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/launch/

cd ~/catkin_ws/
catkin_make install

source /usr/share/gazebo-9/setup.sh
source /opt/ros/melodic/setup.bash
source devel/setup.bash
