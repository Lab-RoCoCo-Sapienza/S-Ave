#include <iostream>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>

#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <srrg_types/types.hpp>


/* PUBLISH IMAGE */

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>


//static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
//static const std::string PUBLISH_TOPIC = "/camera_image";


//#include <fstream>
//#include <yaml-cpp/yaml.h>
//#include <gazebo_msgs/ModelStates.h>
//#include <actionlib/client/simple_action_client.h>
//#include <move_base_msgs/MoveBaseAction.h>
//#include <geometry_msgs/Twist.h>

//#include <queue>
//#include "srrg_path_map/path_map.h"
//#include "srrg_path_map/path_map_utils.h"

using namespace srrg_core;
using namespace Eigen;
//using namespace sensor_msgs;

//namespace enc = sensor_msgs::image_encodings;

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseGoal> MoveBaseClient;

geometry_msgs::PoseStamped start_pose;
geometry_msgs::PoseStamped goal_pose;


void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
	ROS_INFO("Subscribed to 'map' topic");
}


void MoveBaseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  	ROS_INFO("Subscribed to 'move_base_simple/goal' topic");
}
    


bool receiveOccupancyGridMsg(float &resolution,
                             Eigen::Vector2f& origin,
                             srrg_core::UnsignedCharImage &occupancy_grid){

  boost::shared_ptr<nav_msgs::OccupancyGrid const> occupancy_grid_msg_ptr;
  occupancy_grid_msg_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid> ("map", ros::Duration (10));
	
  if(!occupancy_grid_msg_ptr){
    ROS_ERROR("No occupancy_grid message received!");
    return false;
  }

  resolution = occupancy_grid_msg_ptr->info.resolution;
  origin << occupancy_grid_msg_ptr->info.origin.position.x,occupancy_grid_msg_ptr->info.origin.position.y;

  //convert to cv::Mat
  int width = occupancy_grid_msg_ptr->info.width;
  int height = occupancy_grid_msg_ptr->info.height;
  occupancy_grid.create(height,width);

  return true;
}


bool listenRobotPose(Eigen::Isometry3f &robot_pose){
  tf::TransformListener listener;
  tf::StampedTransform robot_tf;
  try {
    listener.waitForTransform("map",
                              "base_link",
                              ros::Time(0),
                              ros::Duration(100));
    listener.lookupTransform("map",
                             "base_link",
                             ros::Time(0),
                             robot_tf);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  robot_pose.setIdentity();
  robot_pose.translation().x() = robot_tf.getOrigin().x();
  robot_pose.translation().y() = robot_tf.getOrigin().y();
  robot_pose.translation().z() = robot_tf.getOrigin().z();
  Eigen::Quaternionf q;
  tf::Quaternion tq = robot_tf.getRotation();
  q.x() = tq.x();
  q.y() = tq.y();
  q.z() = tq.z();
  q.w() = tq.w();
  robot_pose.linear() = q.toRotationMatrix();

  return true;
}


bool getCameraTf(Eigen::Isometry3f &camera){
  tf::TransformListener listener_c;
  tf::StampedTransform camera_tf;
  try {
    listener_c.waitForTransform("camera_link",
                              "base_link",
                              ros::Time(0),
                              ros::Duration(10));
    listener_c.lookupTransform("camera_link",
                             "base_link",
                             ros::Time(0),
                             camera_tf);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  camera.setIdentity();
  camera.translation().x() = camera_tf.getOrigin().x();
  camera.translation().y() = camera_tf.getOrigin().y();
  camera.translation().z() = camera_tf.getOrigin().z();
  Eigen::Quaternionf q;
  tf::Quaternion tq = camera_tf.getRotation();
  q.x() = tq.x();
  q.y() = tq.y();
  q.z() = tq.z();
  q.w() = tq.w();
  camera.linear() = q.toRotationMatrix();

  return true;
}


geometry_msgs::Transform eigen2poseMsg(const Eigen::Isometry3f camera_tf){
    geometry_msgs::Transform p;
    p.translation.x = camera_tf.translation()[0];
    p.translation.y = camera_tf.translation()[1];
    p.translation.z = camera_tf.translation()[2];

    Eigen::Quaternionf q = (Eigen::Quaternionf)camera_tf.linear();
  	p.rotation.x = q.x();
  	p.rotation.y = q.y();
  	p.rotation.z = q.z();
  	p.rotation.w = q.w();

    return p;
 }



// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"simulated_platform_node");
  ros::NodeHandle nh;

  Eigen::Isometry3f camera_transform = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
  Eigen::Vector2f origin = Eigen::Vector2f::Zero();
  srrg_core::UnsignedCharImage occupancy_grid;
  float resolution = 0.0f;

  /* SUBSCRIPTION to 'map' topic */
  ros::Subscriber map_sub = nh.subscribe("map", 10, MapCallback);

  if(listenRobotPose(robot_pose)){
      ROS_INFO("Received robot pose!");
      std::cerr << "Position: " << robot_pose.translation().head(2).transpose() << std::endl;
      std::cerr << "Orientation: " << robot_pose.linear().eulerAngles(0,1,2).z() << std::endl;
  }

  if (getCameraTf(camera_transform)){
  	ROS_INFO("Received camera transformation matrix!");
	std::cout << "Translation: " << std::endl << camera_transform.translation() << std::endl;
	std::cout << "Rotation: " << std::endl << camera_transform.linear() << std::endl;
  }

  if(receiveOccupancyGridMsg(resolution,origin,occupancy_grid)){
    ROS_INFO("Received occupancy grid!");
    std::cerr << "Resolution: " << resolution << std::endl;
    std::cerr << "Dimensions: " << occupancy_grid.rows << "x" << occupancy_grid.cols << std::endl;
    std::cerr << "Origin: " << origin.transpose() << std::endl;
  }

  /* SUBSCRIPTION to 'move_base_simple/goal' topic */
  ros::Subscriber move_base_sub = nh.subscribe("move_base_simple/goal", 10, MoveBaseCallback);

  boost::shared_ptr<geometry_msgs::PoseStamped const> goal_msg_ptr;
  goal_msg_ptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped> ("move_base_simple/goal", ros::Duration (50));
	
  /* SERVICE 'move_base/make_plan' */
  start_pose.header.seq = 0;
  start_pose.header.stamp = ros::Time(0);
  start_pose.header.frame_id = "map";
  start_pose.pose.position.x = robot_pose.translation()[0];
  start_pose.pose.position.y = robot_pose.translation()[1];
  start_pose.pose.position.z = 0.0;
  start_pose.pose.orientation.x = 0.0;
  start_pose.pose.orientation.y = 0.0;
  start_pose.pose.orientation.w = 1.0;

  goal_pose.header.seq = 0;
  goal_pose.header.stamp = ros::Time(0);
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = goal_msg_ptr->pose.position.x;
  goal_pose.pose.position.y = goal_msg_ptr->pose.position.y;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.0;
  //goal_pose.pose.orientation.z = 0.0; //get_plan_msg_ptr.pose.orientation.z;
  goal_pose.pose.orientation.w = 1.0; //get_plan_msg_ptr.pose.orientation.w;


  ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan",true);
  if (!check_path) {
    ROS_FATAL("Could not initialize get plan service from %s",
    check_path.getService().c_str());
    return -1;
  }

  nav_msgs::GetPlan srv;
  srv.request.start = start_pose;
  srv.request.goal = goal_pose;
  srv.request.tolerance = 1.5;

  /* If a plan exists. it is printed the number '1' in the terminal */
  ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));



  /* PUBLISHERS 'camera/rgb/image_raw' and 'camera_tf' */

  // Topic 'camera_tf'
  geometry_msgs::Transform p;
  p = eigen2poseMsg(camera_transform);
  ros::Publisher camera_tf_pub = nh.advertise<geometry_msgs::Transform>("camera_tf", 1000);
  camera_tf_pub.publish(p);
  ROS_INFO("Published 'camera_tf' topic");


  /* NOTE: It is possible to wait for the message coming from 'camera/rgb/image_raw' 
   topic and use it directly */

  // Topic 'camera/rgb/image_raw'
  boost::shared_ptr<sensor_msgs::Image const> image_msg; 
  image_msg = ros::topic::waitForMessage<sensor_msgs::Image> ("/camera/rgb/image_raw", ros::Duration (10));
  ROS_INFO("Published 'camera/rgb/image_raw' topic");

  /*image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("/camera_img", 1);

  ros::Time time = ros::Time::now();
  image_pub.publish(image_msg);
  ROS_INFO("Published 'camera_img' topic");*/


  // Topic 'robot_pose' 
  /*
  ros::Publisher robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose",10);
  robot_pose_pub.publish(start_pose);
  */
  

  ros::spin();

  return 0;
}

