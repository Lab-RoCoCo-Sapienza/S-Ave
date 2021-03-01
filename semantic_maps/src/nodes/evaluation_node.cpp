#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <semantic_maps/ObjectArray.h>
#include <semantic_maps/Object.h>

#include <vector>
#include <string>
#include <cmath>

#include <iostream>
#include <fstream>


// CHANGE
float total_n_objects = 2.0f; // total number of objects in the chosen scene


/*
Object message has the following fields that are necessary to compute ORI:
string label         # class of the object
float32 probability  # detection probability
float32 distance     # robot-object distance
float32 hgazebo      # height of the bounding box in Gazebo
float32 wgazebo      # width of the bounding box in Gazebo
float32 hyolo        # height of Yolo bounding box 
float32 wyolo        # width of the bounding box in Gazebo
int32 count      

To obtain the value of one of the fields, we need to do: 
obj.(name_of_the_field), as for example obj.label or obj.hgazebo
*/
std::vector<float> computeORI(const semantic_maps::ObjectArray::ConstPtr &msg) {
    float ORI = 0.0f;
    float ORI_conf = 0.0f;
    float ORI_scaled = 0.0f;
    std::vector<float> ORI_vector;

    for (int i=0; i<msg->objects.size(); ++i) {
      const semantic_maps::Object &obj = msg->objects[i]; // access the Object at index i
      std::string label_i = msg->objects[i].label;
      std::string taxonomy_i = msg->objects[i].taxonomy;
      float h_y_obj_i = msg->objects[i].hyolo;
	    float w_y_obj_i = msg->objects[i].wyolo;
	    float h_g_obj_i = msg->objects[i].hgazebo;
	    float w_g_obj_i = msg->objects[i].wgazebo;
	    float confidence_i = msg->objects[i].probability;
	    float distance_i = msg->objects[i].distance;
	    float count_i = msg->objects[i].count;
	
	    float Area_y = h_y_obj_i * w_y_obj_i;
	    float Area_g = h_g_obj_i * w_g_obj_i;
      
	    float scaling_factor_i = distance_i/1000.0f;

	    float ori_i = pow(Area_y,2.0f) / pow(Area_g,2.0f);
	    float ori_conf = ori_i * confidence_i; //* 100.0f;
	    float ori_scaled = ori_i * scaling_factor_i;

	    ORI = ORI + ori_i;
	    ORI_conf = ORI_conf + ori_conf;
	    ORI_scaled = ORI_scaled + ori_scaled;

      ROS_INFO("      Object %d", i);
      ROS_INFO("Label: %s", label_i.c_str());
      ROS_INFO("Taxonomy: %s", taxonomy_i.c_str());
      ROS_INFO("Probability: %.3f", confidence_i);
      ROS_INFO("Distance: %.3f", distance_i);
      ROS_INFO("Height_Gazebo: %.3f", h_g_obj_i);
      ROS_INFO("Width_Gazebo: %.3f", w_g_obj_i);
      ROS_INFO("Height_Yolo: %.3f", h_y_obj_i);
      ROS_INFO("Width_Yolo: %.3f", w_y_obj_i);
      ROS_INFO("Number_of_detected_objects: %.3f", count_i);

      ROS_INFO("\n");
      ROS_INFO("ORI_Area: %.3f", ori_i);
      ROS_INFO("ORI_Confidence: %.3f", ori_conf);
      ROS_INFO("ORI_Scaled: %.3f", ori_scaled);
      ROS_INFO("\n");

  }
  ROS_INFO("      FINAL_RESULTS");
  ROS_INFO("ORI_Area: %.3f", ORI);
  ROS_INFO("ORI_Confidence: %.3f", ORI_conf);
  ROS_INFO("ORI_Scaled: %.3f", ORI_scaled);


  ORI_vector.push_back(ORI);
  ORI_vector.push_back(ORI_conf);
  ORI_vector.push_back(ORI_scaled);

  return ORI_vector;
}


/*float computeONI(int n_detected) {
  float oni = (float) (n_detected / total_n_objects);
  ROS_INFO("ONI: %.3f", oni);
  ROS_INFO("\n");
  return oni;
}*/


void ListObjCallback(const semantic_maps::ObjectArray::ConstPtr &msg) {
  ROS_INFO("Subscribed to 'objects_taxonomy' topic");

	int idx_last_obj = msg->objects.size()-1;
	int n_detected = msg->objects[idx_last_obj].count; // number of detected objects
	ROS_INFO("Total number of detected objects: %d", n_detected);
	std::vector<float> ori_vec = computeORI(msg);
	
  //float oni = computeONI(n_detected);
}



// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"evaluation_node");
  ros::NodeHandle nh;

  ros::Subscriber list_obj_sub = nh.subscribe("objects_taxonomy", 10, ListObjCallback);


  /* PUBLISHERS */

  //ros::Publisher camera_tf_pub = nh.advertise<geometry_msgs::Transform>("camera_tf", 10);
  //camera_tf_pub.publish(p);
  //ROS_INFO("Published 'camera_tf' topic");
  

  ros::spin();

  return 0;
}

