#!/bin/sh

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export TURTLEBOT3_MODEL=waffle;
roslaunch turtlebot3_gazebo turtlebot3_world.launch world_file:=$(pwd)/../../src/map/training_area.world x_pos:=0 y_pos:=0" & 
sleep 10

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export TURTLEBOT3_MODEL=waffle;
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(pwd)/../../src/map/training_area.yaml initial_pose_a:=-1.57 initial_pose_x:=0 initial_pose_y:=0 open_rviz:=false" & 
sleep 15

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch home_service home_service.launch " &
sleep 10

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosparam load $(pwd)/../../src/configs/location_config.yaml
rosrun add_markers add_markers " &
sleep 10

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosparam load $(pwd)/../../src/configs/location_config.yaml
rosrun pick_objects pick_objects "
