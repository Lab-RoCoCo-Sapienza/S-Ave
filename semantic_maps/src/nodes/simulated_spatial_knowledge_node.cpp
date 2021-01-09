#include <ros/ros.h>
#include <ros/package.h>

//#include <iostream>
//#include <fstream>

// Messages 
//#include <nav_msgs/OccupancyGrid.h>
//#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/Image.h>

// Include image
//#include <image_transport/image_transport.h>

// Include tf
//#include <tf/tf.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>
//#include <tf/transform_broadcaster.h>

// Include pcl
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// Include message_filters
/*#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <object_detector/object_detector.h>
#include <Eigen/Geometry>
#include <srrg_types/types.hpp>

using namespace srrg_core;
using namespace Eigen;*/


void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
	ROS_INFO("Subscribed to '/camera/depth/points' topic");
}


/*void DetectedObjectsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  	ROS_INFO("Subscribed to 'detected_objects' topic");  	
}*/




// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"simulated_spatial_knowledge_node");
  ros::NodeHandle nh;


  

  /* SUBSCRIPTION to 'detected_objects' topic */
  //ros::Subscriber det_obj_sub = nh.subscribe("detected_objects", 10, DetectedObjectsCallback);


  /* SUBSCRIPTION to '/camera/depth/points' topic */
  ros::Subscriber point_cloud_sub = nh.subscribe("/camera/depth/points", 10, PointCloudCallback);

  
  /* SERVICE get_octomap_of_object */



  /* SERVICE get_detected_objects */


  /* PUBLISHER 'exploration_status' */

  

  ros::spin();

  return 0;
}

