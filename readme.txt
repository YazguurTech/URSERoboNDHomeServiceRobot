## Set it up for the project

sudo apt-get install xterm

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping

git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/catkin_ws/
source devel/setup.bash

rosdep -i install gmapping
rosdep -i install turtlebot3_teleop
rosdep -i install turtlebot3_gazebo

catkin_make
source devel/setup.bash

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

rosrun map_server map_saver -f ~/map






git clone https://github.com/turtlebot/turtlebot_interactions

rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo


rosdep update
rosdep install turtlebot_navigation
sudo apt-get install ros-noetic-turtlebot-navigation




touch launch.sh
touch test_slam.sh
sudo chmod +x launch.sh
sudo chmod +x test_slam.sh

./launch.sh
./test_slam.sh


https://www.youtube.com/watch?v=KJH48r92c9Q




https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/
https://www.turtlebot.com/
https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
https://clearpathrobotics.com/turtlebot-4/
https://github.com/ROBOTIS-GIT/turtlebot3
https://github.com/ROBOTIS-GIT/turtlebot3_simulations



## Set it up for the project

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/turtlebot/turtlebot
git clone https://github.com/turtlebot/turtlebot_interactions
git clone https://github.com/turtlebot/turtlebot_simulator
cd ~/catkin_ws/
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash



rosdep update
rosdep install turtlebot_navigation
sudo apt-get install ros-noetic-turtlebot-navigation


sudo apt-get install xterm

touch launch.sh
touch test_slam.sh
sudo chmod +x launch.sh
sudo chmod +x test_slam.sh

./launch.sh
./test_slam.sh


https://www.youtube.com/watch?v=KJH48r92c9Q
https://knowledge.udacity.com/questions/371403
https://knowledge.udacity.com/questions/371403


$ sudo apt-get install ros-noetic-controller-manager



Create a pick_objects package:
cd ~/catkin_ws/src
catkin_create_pkg pick_objects move_base_msgs actionlib roscpp 


Create a pick_objects package:
cd ~/catkin_ws/src
catkin_create_pkg add_markers visualization_msgs roscpp 

Create a home_service package:
cd ~/catkin_ws/src
catkin_create_pkg home_service 


get a location and goal:
rostopic echo /initialpose
OR
rostopic echo /move_base_simple/goal


#!/bin/sh
xterm  -e  " export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_gazebo turtlebot3_world.launch world_file:=$(pwd)/../../src/map/training_area.world 
" &
sleep 5
xterm  -e  " export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_slam turtlebot3_gmapping.launch " &
sleep 5
xterm  -e  " export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml " &
sleep 5
xterm  -e  " export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch " 


Problems:
1. Costmap location was misplaced - Solution: specify: map_file:=$HOME/map.yaml for turtlebot3 navigation
2. Robot was located in a different place. To fix the issue, update resolution from 0.05 to 0.01 in training_area.yaml file.
3. Map was rotated different angle. <param name="initial_pose_a" value="-1.57" />

