#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <semantic_maps/ObjectArray.h>
#include <semantic_maps/Object.h>

#include <vector>
#include <string>

// CHANGE
const int total_n_objects = 2; // total number of objects in the chosen scene



float computeONI(int n_detected) {
  float oni = n_detected / total_n_objects;
}

/*
Object message has the following fields that are necessary to compute ORI:
string label         # class of the object
float32 probability  # detection probability
float32 threshold    # probability threshold
float32 distance     # robot-object distance
float32 hgazebo      # height of the bounding box in Gazebo
float32 wgazebo      # width of the bounding box in Gazebo
float32 hyolo        # height of Yolo bounding box 
float32 wyolo        # width of the bounding box in Gazebo
int32 count      

To obtain the value of one of the fields, we need to do: 
obj.(name_of_the_field), as for example obj.label or obj.hgazebo
*/
float computeORI(const semantic_maps::ObjectArray::ConstPtr &msg) {
  for (int i=0; i<msg->objects.size(); ++i) {
      const semantic_maps::Object &obj = msg->objects[i]; // access the Object at index i
      
  }
  return 0.0;
}


void ListObjCallback(const semantic_maps::ObjectArray::ConstPtr &msg) {
	ROS_INFO("Subscribed to 'detected_objects' topic");

  int idx_last_obj = msg->objects.size()-1;
  int n_detected = msg->objects[idx_last_obj].count; // number of detected objects
  ROS_INFO("Total number of detected objects: %d", n_detected);
  float oni = computeONI(n_detected);
  float ori = computeORI(msg);
  
}








// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"evaluation_node");
  ros::NodeHandle nh;

  ros::Subscriber list_obj_sub = nh.subscribe("detected_objects", 10, ListObjCallback);


  /* PUBLISHERS */

  //ros::Publisher camera_tf_pub = nh.advertise<geometry_msgs::Transform>("camera_tf", 10);
  //camera_tf_pub.publish(p);
  //ROS_INFO("Published 'camera_tf' topic");
  

  ros::spin();

  return 0;
}

