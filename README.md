# S-Ave

## Installation
In this section, we provide the commands needed to download the required packages that must be located in the **catkin_ws/src** folder:
* *semantic_maps*: `git clone https://github.com/Lab-RoCoCo-Sapienza/S-Ave.git`
* *turtlebot3*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`
* *turtlebot3_msgs*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`
* *turtlebot3_simulations*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git` </br>
</br>

The folder **srrg** (containing 3 needed packages that are listed below) must also be positioned in the **catkin_ws/src** folder:
* *srrg_core*: `git clone https://gitlab.com/srrg-software/srrg_core.git`
* *srrg_core_ros*: `git clone https://gitlab.com/srrg-software/srrg_core_ros.git`
* *srrg_cmake_modules*: `git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git`

## Running the code
**simulated_platform_node**

Open 5 terminal windows and launch the commands here presented by following this order:
  1. `roscore`
  2. `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
  3. `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`
  4. `roslaunch semantic_maps move_base.launch`
  5. `rosrun semantic_maps simulated_platform_node`
  
