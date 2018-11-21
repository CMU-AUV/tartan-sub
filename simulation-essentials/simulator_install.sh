mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
cd src
git clone https://github.com/ros-drivers/video_stream_opencv.git
cd ..
catkin_make


### Simulator 
cd src 
git clone https://github.com/uuvsimulator/uuv_simulator.git

source /usr/share/gazebo-7/setup.sh
source /opt/ros/kinetic/setup.bash
source /devel/setup.bash

source ~/.bashrc

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

cd ~/catkin_ws
catkin_make install

cd ~/tartan-sub/simulation-essentials/robosub-worlds
cp * ~/catkin_ws/src/uuv_simulator/uuv_descriptions/worlds/

cd ..
cd robosub-worlds-models
cp -r * ~/catkin_ws/src/uuv_simulator/uuv_descriptions/world_models

cd ~/catkin_ws/src/
mv -f ~/tartan-sub/ .
cd ..
catkin_make install
