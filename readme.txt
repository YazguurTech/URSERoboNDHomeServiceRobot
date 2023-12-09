## Home service

The Home Service Robot is the final project of the Robotics Software Engineer nanodegree program on the Udacity.
The Home Service Robot simulates a home service robot capable of navigating to pick up and deliver virtual objects.
This readme contains instructions about how to run the project.

### Software requirements:
 - Ubuntu 20.04
 - ROS Noetic
 - Gazebo 11

### Used ROS packages:
 - [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) - TurtleBot is a ROS standard platform robot. There are multiple versions (4 different versions as of December 2023) of the TurtleBot model. 
   We are using Turtlebot3 for this project. TurtleBot3 is a small, affordable, programmable, ROS-based mobile robot for use in education, research, hobby, and product prototyping.
 - [Turtlebot3 simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) - TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. 
   There are two development environments to do this, one is using a fake node with 3D visualization tool RViz, and the other is using the 3D robot simulator Gazebo.

### Other tools:
 - Xterm - xterm is the standard terminal emulator for the X Window System.
    ```
    sudo apt-get install xterm
    ```

### Installing dependences:
    ```
    rosdep -i install gmapping
    rosdep -i install turtlebot3_teleop
    rosdep -i install turtlebot3_gazebo
    ```

    `If you haven't set up environment, please refer to the "Set up environment" section below.`

### Steps to run:
There are the following 5 script files in the src/scripts directory:
    - test_slam.sh - It will deploy a waffle turtlebot3 inside a gazebo environment. You can control it with keyboard commands. 
    It will interface it with a SLAM package, and you can visualize the map in RViz.
    - test_navigation.sh - It will deploy a waffle turtlebot3 inside a gazebo environment. 
    Also, it will launch turtlebot3 navigation where you can manually navigate the robot to reach a desired position using the 2D Nav Goal functionality on the RViz.
    - pick_objects.sh - It will run the pick_objects node which will autonomously send successive goals for the robot to reach.
    - add_markers.sh - It will run the add_markers node which will publish markers in RViz to imitate a virtual object appearing in its pickup zone,
    and then in its drop off zone.
    - home_service.sh - It will run the add_markers and pick_objects node which will simulate a home service robot capable of 
    navigating to pick up and deliver virtual objects.

1. Download and extract the Home Service Robot zip file.
2. Go the URSERoboNDHomeServiceRobot directory.
    ```
    $ cd URSERoboNDHomeServiceRobot
    ```
3. Build the project
    ```
    catkin_make
    source devel/setup.bash
    ```
4. Go a /src/scripts directory on the URSERoboNDHomeServiceRobot folder.
    ```
    $ cd URSERoboNDHomeServiceRobot/src/scripts
    ```
5. Execute the corresponding script file.

    Test SLAM
    ```
    $ ./test_slam.sh
    ```

    Test Navigation
    ```
    $ ./test_navigation.sh 
    ```

    Pick objects
    ```
    $ ./pick_objects.sh 
    ```

    Add markers
    ```
    $ ./add_markers.sh
    ```

    Home Service
    ```
    $ ./home_service.sh
    ```

### Setting up the environment

#### Install ROS Noetic 
 ```
 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt install curl
 curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 sudo apt update
 sudo apt install ros-noetic-desktop-full
 echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
 sudo apt install python3-rosdep
 sudo rosdep init
 rosdep update
 ```

 #### Install gazebo
 ```
 curl -sSL http://get.gazebosim.org | sh
 ```

 #### Install packages
 ```
 sudo apt-get install ros-noetic-navigation 
 sudo apt-get install ros-noetic-map-server 
 sudo apt-get install ros-noetic-move-base 
 sudo apt-get install ros-noetic-amcl
 sudo apt-get install libignition-math4-dev 
 sudo apt-get install protobuf-compiler
 sudo apt-get install ros-noetic-rtabmap-ros
 ```

 #### Install Visual Studio Code
 ```
 sudo apt-get install wget gpg
 wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
 sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
 sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
 rm -f packages.microsoft.gpg
 sudo apt install apt-transport-https
 sudo apt update
 sudo apt install code
 ```
