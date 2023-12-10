## Home service

The Home Service Robot is the final project of the Robotics Software Engineer nanodegree program on the Udacity.
The Home Service Robot simulates a home service robot capable of navigating to pick up and deliver virtual objects.
This readme contains instructions about how to run the project.

### Software requirements
 - Ubuntu 20.04
 - ROS Noetic
 - Gazebo 11

### ROS packages
 - [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) - TurtleBot is a ROS standard platform robot. There are multiple versions (4 different versions as of December 2023) of the TurtleBot model. We are using Turtlebot3 for this project. TurtleBot3 is a small, affordable, programmable, ROS-based mobile robot for use in education, research, hobby, and product prototyping.
 - [Turtlebot3 simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) - TurtleBot3 supports simulation development environment that can be programmed and developed with a virtual robot in the simulation. There are two development environments to do this, one is using a fake node with 3D visualization tool RViz, and the other is using the 3D robot simulator Gazebo.

### Other tools
 - Xterm - xterm is the standard terminal emulator for the X Window System.
    ```
    sudo apt-get install xterm
    ```

### Installing dependences

    ```
    rosdep -i install gmapping
    rosdep -i install turtlebot3_teleop
    rosdep -i install turtlebot3_gazebo
    ```
    If you haven't set up environment, please refer to the "Setting up the environment" section below.

### Packages Used

In the project, the integration of custom packages — home service, pick objects, and add markers alongside the foundational ROS packages turtlebot3 and turtlebot3 simulations, with a focus on turtlebot3 navigation, forms a  framework for simulating a home service robot. This system is to efficiently navigate within an environment, simulating the seamless pickup and delivery of virtual objects.

#### Custom Packages

- **Home service** - It simplifies simulation setup with an RViz configuration and launch file.
- **Pick objects**: - It i responsible for precise navigation, pick objects enables the robot to autonomously navigate to pre-configured pick-up and drop-off locations. It leverages the capabilities of the ROS Navigation Stack to plan and execute trajectories.
- **Add markers**: It subscribes to odometry, keeping track of the robot's position, and publishes markers, enhancing visualization in RViz. It provides real-time feedback on the robot's location and status.

#### Foundational ROS Packages

- **turtlebot3 and turtlebot3 simulations** - It provides simulation environments and tools for TurtleBot3 robots. A custom Gazebo world was utilized in the project.
- **turtlebot3 acml** - Adaptive Monte Carlo Localization (AMCL) is employed through this package to enable Simultaneous Localization and Mapping (SLAM) capabilities. AMCL allows the robot to create and update a map of its environment while estimating its pose with the help of a particle filter.
- **turtlebot3 navigation** - It enables navigation capabilities for TurtleBot3 robots, integrating with the AMCL (Adaptive Monte Carlo Localization) algorithm and move base. AMCL is employed for robot localization, while move base facilitates path planning and obstacle avoidance during navigation.

#### Key Localization and Mapping Packages

 - **AMCL (Adaptive Monte Carlo Localization)** - AMCL is a probabilistic localization algorithm that utilizes a particle filter to estimate the robot's pose within an environment. 
 - **GMapping** - Grid-based Mapping is integral for Simultaneous Localization and Mapping (SLAM). It uses laser scan data to create a 2D occupancy grid map of the environment while concurrently estimating the robot's pose.

 #### Navigation Stack Component
  - **Move Base** - It is a central component that brings together the navigation stack. It integrates global and local planners, a costmap generator, and controllers, guiding the robot from its current position to a specified goal while avoiding obstacles.

### Source folder structure

- **home_service** - It contains the rviz configurations and the launch file.
- **add_markers** - It contains add_markers.cpp, add_markers_testonly.cpp, as well as the RViz and launch files associated with add_markers_testonly. add_markers.cpp is utilized for home service simulation, while add_markers_testonly.cpp is specifically employed for testing marker functionality in isolation.
- **pick_objects** - It contains pick_objects.cpp and pick_objects_testonly.cpp. pick_objects.cpp is utilized for home service simulation, while pick_objects_testonly.cpp is specifically employed for testing navigation functionality in isolation.
- **configs** - It includes location_config.xml, where the pickup and drop-off zone locations are configured.
- **map** - It contains the Gazebo world file, the map file, and their respective configurations.
- **turtlebot3** - It includes files related to Turtlebot3, encompassing various configurations, launch files, and other associated components.
- **turtlebot3_simulations** - It contains files related to Turtlebot3 simulations.
- **scripts**  There are the following 5 script files in the scripts directory:
  - **test_slam.sh** - It will deploy a waffle turtlebot3 inside a gazebo environment. You can control it with keyboard commands. 
 It will interface it with a SLAM package, and you can visualize the map in RViz.
  - **test_navigation.sh** - It will deploy a waffle turtlebot3 inside a gazebo environment. Also, it will launch turtlebot3 navigation where you can manually navigate the robot to reach a desired position using the 2D Nav Goal functionality on the RViz.
  - **pick_objects.sh** - It will run the pick_objects node which will autonomously send successive goals for the robot to reach.
  - **add_markers.sh** - It will run the add_markers node which will publish markers in RViz to imitate a virtual object appearing in its pickup zone, and then in its drop off zone.
  - **home_service.sh** - It will run the add_markers and pick_objects node which will simulate a home service robot capable of navigating to pick up and deliver virtual objects.

### Steps to run

1. Download and extract the HomeServiceRobot.zip file.
2. Go the HomeServiceRobot directory.
    ```
    cd HomeServiceRobot
    ```
3. Build the project
    ```
    catkin_make
    source devel/setup.bash
    ```
4. Go a src/scripts directory on the HomeServiceRobot folder.
    ```
    cd src/scripts/
    ```
5. Execute the corresponding script file.

    Test SLAM
    ```
    ./test_slam.sh
    ```
    Test Navigation
    ```
    ./test_navigation.sh 
    ```
    Pick objects
    ```
    ./pick_objects.sh 
    ```
    Add markers
    ```
    ./add_markers.sh
    ```
    Home Service
    ```
    ./home_service.sh
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
