# S-Ave

## Installation
In this section, we provide the commands needed to download the required packages that must be located in the **catkin_ws/src** folder:
* *S-Ave*: `git clone https://github.com/Lab-RoCoCo-Sapienza/S-Ave.git`
* *octomap_mapping*: `git clone https://github.com/OctoMap/octomap_mapping.git`
* *turtlebot3*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`
* *turtlebot3_msgs*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`
* *turtlebot3_simulations*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git` </br>
</br>

### srrg
The folder **srrg** (containing 3 needed packages that are listed below) must also be positioned in the **catkin_ws/src** folder:
* *srrg_core*: `git clone https://gitlab.com/srrg-software/srrg_core.git`
* *srrg_core_ros*: `git clone https://gitlab.com/srrg-software/srrg_core_ros.git`
* *srrg_cmake_modules*: `git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git` </br>
</br>


### ncnn
The **ncnn** package is needed for *yolov5_object_detection_node*, but in order not to encouter any type of error during compilation, it is required to follow these steps: </br>
`$ git clone https://github.com/Tencent/ncnn.git` </br>
`$ cd ncnn` </br>
`$ git submodule update --init` </br>

To build the library, it is necessary to run these commands: </br>
`$ cd ncnn` </br>
`$ mkdir -p build` </br>
`$ cd build` </br>
`build$ cmake -DCMAKE_BUILD_TYPE=Release -DNCNN_VULKAN=OFF -DNCNN_SYSTEM_GLSLANG=ON -DNCNN_BUILD_EXAMPLES=ON ..` </br>
`build$ make -j$(nproc)` </br>

**NOTE**: Have a look at *https://github.com/Tencent/ncnn/wiki/how-to-build#build-for-linux-x86* if you want to install and use Vulcan. </br> 

Once ncnn library is compiled, we need to move into **catkin/src** folder, create the **ncnn** package with the following command: </br>
`catkin_create_pkg ncnn`
and move the entire library (the folder downloaded with the command `git clone https://github.com/Tencent/ncnn.git`) into this package.

### sudo apt-get
In order to move the robot, the **teleop_twist_keyboard** package is required, so to install it, open a terminal and launch the command: </br> 
`sudo apt-get install ros-melodic-teleop-twist-keyboard` </br>

Both, the **octomap** package and **rviz-plugins** are also required, so to install them, open a terminal and launch the following commands one after the other:
* `sudo apt-get install ros-melodic-octomap-ros` 
* `sudo apt-get install ros-melodic-octomap-msgs` 
* `sudo apt-get install ros-melodic-octomap-server` 
* `sudo apt-get install ros-melodic-octomap-rviz-plugins` </br>


Once all the packages have been downloaded, move to **catkin_ws** folder and run the command `catkin_make` to compile the code. </br>

## Running the code
**platform_node** and **yolov5_object_detector_node**

Open 7 terminal windows and launch the commands here presented by following this order:
  1. `roslaunch semantic_maps gazebo_robot.launch`
  2. `roslaunch semantic_maps gmapping.launch`
  3. `roslaunch semantic_maps move_base.launch`
  4. `roslaunch semantic_maps rviz_robot.launch` -> (*if some options on the left of the visualizer appear in red, close Rviz and run again the command*)
  5. `rosrun semantic_maps platform_node`
  6. `roslaunch semantic_maps yolov5l.launch`
  7. `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/lucrezio/cmd_vel`
  
  
