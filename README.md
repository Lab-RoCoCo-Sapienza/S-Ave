# S-Ave

## Environment
* Ubuntu 18.04
* ROS melodic

## Installation
Firstly, it is necessary to create the workspace, so open a terminal and type with the following command: </br>
`$ mkdir -p ~/catkin_ws/src` </br>

Type the following command to edit the .bashrc text file: </br>
`$ gedit ~/.bashrc` </br>

Add this line to the end of the .bashrc file: </br>
`$ source ~/catkin_ws/devel/setup.bash` </br>

Now, we provide the commands needed to download the required packages that must be located in the **catkin_ws/src** folder created before:
* *S-Ave*: `git clone https://github.com/Lab-RoCoCo-Sapienza/S-Ave.git`
* *octomap_mapping*: `git clone https://github.com/OctoMap/octomap_mapping.git`
* *turtlebot3*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`
* *turtlebot3_msgs*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`
* *turtlebot3_simulations*: `git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git` </br>

### srrg
The folder **srrg** (containing 3 needed packages that are listed below) must also be positioned in **catkin_ws/src** folder:
* *srrg_core*: `git clone https://gitlab.com/srrg-software/srrg_core.git`
* *srrg_core_ros*: `git clone https://gitlab.com/srrg-software/srrg_core_ros.git`
* *srrg_cmake_modules*: `git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git` </br>

### ncnn
The **ncnn** package is needed for *yolov5_object_detection_node*, but in order not to encouter any type of error during compilation, it is required to move into **catkin/src** folder and follow these steps:
  1. `$ catkin_create_pkg ncnn` (it creates a ros-package with the name *ncnn*) </br>
  2. `$ cd ncnn` </br>
  3. `ncnn$ mkdir -p src` </br>
  4. `ncnn$ cd src` </br>
  5. `ncnn/src$ git clone https://github.com/Tencent/ncnn.git` </br>
  6. `ncnn/src$ cd ncnn` </br>
  7. `ncnn/src/ncnn$ git submodule update --init` </br>

To build the library, it is necessary to remain in the 'inner' ncnn folder (it is a library) and run these commands: </br>
  8. `ncnn/src/ncnn$ mkdir -p build` </br>
  9. `ncnn/src/ncnn$ cd build` </br>
  10. `ncnn/src/ncnn/build$ cmake -DCMAKE_BUILD_TYPE=Release -DNCNN_VULKAN=OFF -DNCNN_SYSTEM_GLSLANG=ON -DNCNN_BUILD_EXAMPLES=ON ..` </br>
  11. `ncnn/src/ncnn/build$ make -j$(nproc)` </br>
  12. `ncnn/src/ncnn/build$ make install` </br>

Now we need to allow the program to see the libncnn.a library, so in order to do that, remain in the **build** folder and launch in the terminal: </br>
  13. `ncnn/src/ncnn/build$ sudo cp -r install/include/ncnn /usr/local/include/ncnn` </br>
  14. `ncnn/src/ncnn/build$ sudo cp -r install/lib/libncnn.a /usr/local/lib/libncnn.a` </br>

**NOTE**: Have a look at *https://github.com/Tencent/ncnn/wiki/how-to-build#build-for-linux-x86* if you want to install and use Vulcan. </br> 

### sudo apt-get install
In order to move the robot, the **teleop_twist_keyboard** package is required, so to install it, open a terminal and launch the command: </br> 
`sudo apt-get install ros-melodic-teleop-twist-keyboard` </br>

Both, the **octomap** package and **rviz-plugins** are also required, so to install them, open a terminal and launch the following commands one after the other:
* `sudo apt-get install ros-melodic-octomap-ros` 
* `sudo apt-get install ros-melodic-octomap-msgs` 
* `sudo apt-get install ros-melodic-octomap-server` 
* `sudo apt-get install ros-melodic-octomap-rviz-plugins` </br>

Another important package to install is **slam-gmapping**: </br>
`sudo apt-get install ros-melodic-slam-gmapping` </br>

Once all the packages have been downloaded, move to **catkin_ws** workspace and run the command `catkin_make` to compile the code by launching the following command: </br>
`$ cd ~/catkin_ws && catkin_make`

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
  
  
