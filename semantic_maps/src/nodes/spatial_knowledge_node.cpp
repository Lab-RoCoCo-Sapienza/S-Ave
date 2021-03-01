#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <semantic_maps/Object.h>
#include <semantic_maps/ObjectArray.h>

#include <vector>
#include <string>
#include <cmath>
#include <map>

#include <iostream>
#include <fstream>


semantic_maps::ObjectArray list_obj_msg;
semantic_maps::Object objMsg;
ros::Subscriber detected_objects_sub;
ros::Subscriber point_cloud_sub;
ros::Publisher list_obj_pub;


// taxonomy dictionary
std::map<std::string, std::string> taxonomy_dict = 
{
    { "couch", "furniture" },
    { "dining table", "furniture" },
    { "chair", "furniture" },
    { "bed", "furniture" },
    { "oven", "applicance" },
    { "sink", "applicance" },
    { "refrigerator", "applicance" },
    { "microwave", "applicance" },
    { "toilet", "applicance" },
    { "laptop", "accessories" },
    { "tv", "accessories" },
    { "book", "accessories" },
};



void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
	ROS_INFO("Subscribed to 'camera/depth/points' topic");
}


void DetectedObjectsCallback(const semantic_maps::ObjectArray::ConstPtr &msg) {
  	ROS_INFO("Subscribed to 'detected_objects' topic");  	

  	for (int i=0; i<msg->objects.size(); ++i) {
        const semantic_maps::Object &obj = msg->objects[i]; // access the Object at index i
        objMsg.header = msg->objects[i].header;
        objMsg.label = msg->objects[i].label;
        objMsg.probability = msg->objects[i].probability;
        objMsg.distance = msg->objects[i].distance;
        objMsg.wyolo = msg->objects[i].wyolo;
        objMsg.hyolo = msg->objects[i].hyolo;
        objMsg.wgazebo = msg->objects[i].wgazebo;
        objMsg.hgazebo = msg->objects[i].hgazebo;
        objMsg.count = msg->objects[i].count;
        objMsg.taxonomy = taxonomy_dict[objMsg.label];

        list_obj_msg.objects.push_back(objMsg);

    }
    list_obj_pub.publish(list_obj_msg);
}


// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"spatial_knowledge_node");
  ros::NodeHandle nh;

  detected_objects_sub = nh.subscribe("/detected_objects", 10, DetectedObjectsCallback);

  //point_cloud_sub = nh.subscribe("/camera/depth/points", 10, PointCloudCallback);

  list_obj_pub = nh.advertise<semantic_maps::ObjectArray>("/objects_taxonomy", 50);


  ros::spin();

  return 0;
}

