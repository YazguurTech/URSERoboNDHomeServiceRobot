#!/bin/sh

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export TURTLEBOT3_MODEL=waffle;
roslaunch turtlebot3_gazebo turtlebot3_world.launch world_file:=$(pwd)/../../src/map/training_area.world x_pos:=0 y_pos:=0" & 
sleep 10


xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export TURTLEBOT3_MODEL=waffle;
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(pwd)/../../src/map/training_area.yaml initial_pose_a:=-1.57 initial_pose_x:=0 initial_pose_y:=0 " & 
sleep 20

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun pick_objects pick_objects_testonly "
