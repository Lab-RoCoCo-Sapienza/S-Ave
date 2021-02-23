#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <semantic_maps/Object.h>


void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
	ROS_INFO("Subscribed to 'camera/depth/points' topic");
}


void DetectedObjectsCallback(const semantic_maps::Object::ConstPtr &msg) {
  	ROS_INFO("Subscribed to 'detected_objects' topic");  	
}


// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"spatial_knowledge_node");
  ros::NodeHandle nh;


  /* SUBSCRIPTION to 'detected_objects' topic */
  ros::Subscriber detected_objects_sub = nh.subscribe("/detected_objects", 50, DetectedObjectsCallback);


  /* SUBSCRIPTION to '/camera/depth/points' topic */
  ros::Subscriber point_cloud_sub = nh.subscribe("/camera/depth/points", 10, PointCloudCallback);

  

  ros::spin();

  return 0;
}

