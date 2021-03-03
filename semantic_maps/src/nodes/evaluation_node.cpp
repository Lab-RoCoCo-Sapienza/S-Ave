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
#include <map>

#include "json.h"

#include "boost/date_time/posix_time/posix_time.hpp"



float total_n_objects = 0.f; // total number of objects in the chosen scene
std::string world_name;
std::string exploration_mode;

std::string iso_time_str = "";
float time_s_init = 0.f;

int counter = 0;


std::map<std::string, float > total_n_objects_map = {
  {"rococo_lab", 21.0},
  {"test_apartment", -1.0},
  {"apartment_2", -1.0},
  {"apartment_4", 35.f},
  {"restaurant", -1.0},
  {"phd_office", 41.0},
  {"prof_office", 11.0},
};

std::map<std::string, std::vector<float> > metrics_dict = {
  {"timestep", {}},
  {"ori_surface", {}},
  {"ori_confidence", {}},
  {"ori_distance", {}},
  {"opi", {}}
};

void appendMetrics(std::map<std::string, std::vector<float> >* reference_dict,
                   const std::map<std::string, float>& metrics) {
  for (auto it = metrics.begin(); it != metrics.end(); ++it) {
    (*reference_dict)[it->first].push_back(it->second);
  }
}

void createMetricsDict(std::map<std::string, float>* metrics_t, float timestep,
                       float ori_surface, float ori_confidence, float ori_distance,
                       float opi, float time_s) {
  metrics_t->clear();
  (*metrics_t) = {
    {"timestep", timestep}, 
    {"time_s", time_s},
    {"ori_surface", ori_surface},
    {"ori_confidence", ori_confidence},
    {"ori_distance", ori_distance},
    {"opi", opi}
  };
}

void saveMetricsToJSON(std::string save_path, const std::map<std::string, std::vector<float> >& metrics) {
  nlohmann::ordered_json json_metrics;
  for (auto it = metrics.begin(); it != metrics.end(); ++it) {
    json_metrics[it->first] = it->second;
  }

  std::ofstream out(save_path);
  out << json_metrics.dump();
  out.close();
}


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

      ROS_INFO("      Object %d", i);
      ROS_INFO("Label: %s", label_i.c_str());
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



float computeONI(int n_detected) {
  float oni = (float) (n_detected / total_n_objects);
  ROS_INFO("ONI: %.3f", oni);
  ROS_INFO("\n");
  return oni;
}

float computeOPI(int n_detected) { // NEW
  float oni = (float) (n_detected / total_n_objects);
  ROS_INFO("ONI: %.3f", oni);
  ROS_INFO("\n");
  return oni;
}



void ListObjCallback(const semantic_maps::ObjectArray::ConstPtr &msg) {
    
    int idx_last_obj = msg->objects.size()-1;
    if (idx_last_obj < 0) return;
    int n_detected = msg->objects[idx_last_obj].count; // number of detected objects

    ROS_INFO("Total number of detected objects: %d", n_detected);
    std::vector<float> ori_vec = computeORI(msg);

    std::map<std::string, float > metrics_t;
    float timestep = ros::Time::now().toSec();
    float time_s = ros::WallTime::now().toSec() - time_s_init;
    float ori_surface = ori_vec.at(0);
    float ori_confidence = ori_vec.at(1);
    float ori_distance = ori_vec.at(3-1);
    float opi = computeOPI(n_detected);

    createMetricsDict(&metrics_t, timestep, ori_surface, ori_confidence, ori_distance, opi, 		time_s);
    appendMetrics(&metrics_dict, metrics_t);


    std::string save_path_relative = ros::package::getPath("semantic_maps");
    std::string save_path = save_path_relative+"/results/"+ exploration_mode +"/"+world_name+"/"+ iso_time_str + ".json";

    ROS_INFO("ITERATION: %i, t-t0(s): %f", ++counter, time_s);

    saveMetricsToJSON(save_path, metrics_dict);

}




// ------------------------------------------------------

int main(int argc, char **argv){

  ros::init(argc,argv,"evaluation_node");
  ros::NodeHandle nh;

  nh.param<std::string>("/evaluation_node/world_name", world_name, "apartment_2");	
  nh.param<std::string>("/evaluation_node/exploration_mode", exploration_mode, "teleop");	
  ROS_INFO("WORLD NAME: %s", world_name.c_str());
  ros::Subscriber list_obj_sub = nh.subscribe("objects_taxonomy", 10, ListObjCallback);

  time_s_init = ros::WallTime::now().toSec();
  boost::posix_time::ptime my_posix_time = ros::WallTime::now().toBoost();
  iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
  counter = 0;

  total_n_objects = total_n_objects_map.at(world_name);


  ros::spin();

  return 0;
}

