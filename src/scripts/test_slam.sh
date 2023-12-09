#!/bin/sh
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export TURTLEBOT3_MODEL=waffle;
roslaunch turtlebot3_gazebo turtlebot3_world.launch world_file:=$(pwd)/../../src/map/training_area.world x_pos:=0 y_pos:=0 " &
sleep 10

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export TURTLEBOT3_MODEL=waffle;
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping " &
sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export TURTLEBOT3_MODEL=waffle;
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch " 