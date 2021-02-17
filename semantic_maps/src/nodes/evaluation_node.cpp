#include <iostream>
#include <ros/ros.h>



/*void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
	ROS_INFO("Subscribed to 'map' topic");
}*/






// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"evaluation_node");
  ros::NodeHandle nh;

  /* SUBSCRIPTIONS */
  //ros::Subscriber map_sub = nh.subscribe("map", 10, MapCallback);


  /* PUBLISHERS */

  //ros::Publisher camera_tf_pub = nh.advertise<geometry_msgs::Transform>("camera_tf", 10);
  //camera_tf_pub.publish(p);
  //ROS_INFO("Published 'camera_tf' topic");
  

  ros::spin();

  return 0;
}

