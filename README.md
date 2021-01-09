# S-Ave

## Installation
In this section, we provide the commands needed to download the required packages that must be located in the **catkin_ws/src** folder:
* *S-Ave*: `git clone https://github.com/Lab-RoCoCo-Sapienza/S-Ave.git`
* *turtlebot3*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`
* *turtlebot3_msgs*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`
* *turtlebot3_simulations*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git` </br>
</br>

The folder **srrg** (containing 3 needed packages that are listed below) must also be positioned in the **catkin_ws/src** folder:
* *srrg_core*: `git clone https://gitlab.com/srrg-software/srrg_core.git`
* *srrg_core_ros*: `git clone https://gitlab.com/srrg-software/srrg_core_ros.git`
* *srrg_cmake_modules*: `git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git` </br>
</br>

In order to move the robot, the **teleop_twist_keyboard** package is required, so to install it, open a terminal and launch the command: </br> 
`sudo apt-get install ros-melodic-teleop-twist-keyboard` </br>

Once all the packages have been downloaded, move to **catkin_ws** folder and run the command `catkin_make` to compile the code. </br>

## Running the code
**simulated_platform_node** and **simulated_ideal_object_detector_node**

Open 7 terminal windows and launch the commands here presented by following this order:
  1. `roslaunch semantic_maps gazebo_robot.launch`
  2. `roslaunch semantic_maps gmapping.launch`
  3. `roslaunch semantic_maps move_base.launch`
  4. `rosrun semantic_maps simulated_platform_node`
  5. `rosrun semantic_maps simulated_ideal_object_detector_node`
  6. `roslaunch semantic_maps rviz_robot.launch`
  7. `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/lucrezio/cmd_vel`
  
  
