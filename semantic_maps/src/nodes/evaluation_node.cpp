#include <iostream>
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

  		/*results << "           Object: " << i << "\n";
  		results << "Label: " << label_i << "\n";
  		results << "Probability: " << confidence_i << "\n";
  		results << "Distance: " << distance_i << "\n";
  		results << "Height_Gazebo: " << h_g_obj_i << "\n";
  		results << "Width_Gazebo: " << w_g_obj_i << "\n";
  		results << "Height_Yolo: " << h_y_obj_i << "\n";
  		results << "Width_Yolo: " << w_y_obj_i << "\n";
  		results << "Number_of_detected_objects: " << count_i << "\n";

  		results << "ORI_Area: " << ori_i << "\n";
  		results << "ORI_Confidence: " << ori_conf << "\n";
  		results << "ORI_Scaled: " << ori_scaled << "\n";
  		results << "\n";
  		results << "\n";*/
  }
  /*results << "\n";
  results << "           FINAL_RESULTS " << "\n";
  results << "ORI_Area: " << ORI << "\n";
  results << "ORI_Confidence: " << ORI_conf << "\n";
  results << "ORI_Scaled: " << ORI_scaled << "\n";*/

  ORI_vector.push_back(ORI);
  ORI_vector.push_back(ORI_conf);
  ORI_vector.push_back(ORI_scaled);

  return ORI_vector;
}


float computeONI(int n_detected) {
  float oni = (float) (n_detected / total_n_objects);
  return oni;
}


void ListObjCallback(const semantic_maps::ObjectArray::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'detected_objects' topic");

	int idx_last_obj = msg->objects.size()-1;
	int n_detected = msg->objects[idx_last_obj].count; // number of detected objects
	ROS_INFO("Total number of detected objects: %d", n_detected);
	std::vector<float> ori_vec = computeORI(msg);
	ROS_INFO("Final ori_vec: %.3f, %.3f, %.3f", ori_vec[0], ori_vec[1], ori_vec[2]);
	float oni = computeONI(n_detected);
	ROS_INFO("Final oni: %.3f", oni);
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

