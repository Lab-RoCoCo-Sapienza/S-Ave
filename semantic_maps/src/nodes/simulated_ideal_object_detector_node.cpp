#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>

// Messages
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <semantic_maps/LogicalImage.h>

// Include image
#include <image_transport/image_transport.h>

// Include tf
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Include pcl
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// Include message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <object_detector/object_detector.h>
#include <Eigen/Geometry>

using namespace Eigen;


class IdealObjectDetectorNode{

public:
  IdealObjectDetectorNode(ros::NodeHandle nh_):
    _nh(nh_),
    _logical_image_sub(_nh,"/gazebo/logical_camera_image",1),  //move to SKn node
    _map_sub(_nh,"/map",1), // subscription to map topic
    _depth_points_sub(_nh,"/camera/depth/points",1),  // subscription to camera/depth/points topic
    _camera_tf_sub(_nh,"/camera_tf",1),  // subscription to camera_tf topic
    _synchronizer(FilterSyncPolicy(1000),_logical_image_sub,_depth_points_sub),
    _it(_nh){

      _synchronizer.registerCallback(boost::bind(&IdealObjectDetectorNode::filterCallback, this, _1, _2));

      _camera_offset.setIdentity();
      _camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();

    ROS_INFO("Running simulated_ideal_object_detector_node...");
  }


  void filterCallback(const semantic_maps::LogicalImage::ConstPtr &logical_image_msg,
                      const PointCloud::ConstPtr &depth_points_msg){

    ROS_INFO("Executing the Callback...");

    //get camera pose
    _camera_transform = poseMsg2eigen(logical_image_msg->pose);
  
    //get point cloud
    PointCloud::Ptr transformed_cloud (new PointCloud ());
    pcl::transformPointCloud (*depth_points_msg, *transformed_cloud, _camera_transform*_camera_offset);
  }


protected:

  //ros stuff
  ros::NodeHandle _nh;

  //synchronized subscriber to rgbd frame and logical_image
  message_filters::Subscriber<semantic_maps::LogicalImage> _logical_image_sub;
  message_filters::Subscriber<PointCloud> _depth_points_sub;
  message_filters::Subscriber<nav_msgs::OccupancyGrid> _map_sub;
  message_filters::Subscriber<geometry_msgs::Transform> _camera_tf_sub;
  typedef message_filters::sync_policies::ApproximateTime<semantic_maps::LogicalImage,
  PointCloud> FilterSyncPolicy;
  message_filters::Synchronizer<FilterSyncPolicy> _synchronizer;

  Eigen::Isometry3f _camera_transform;
  Eigen::Isometry3f _camera_offset;

  image_transport::ImageTransport _it;


private:

  Eigen::Isometry3f poseMsg2eigen(const geometry_msgs::Pose& p){
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
    iso.translation().x() = p.position.x;
    iso.translation().y() = p.position.y;
    iso.translation().z() = p.position.z;
    Eigen::Quaternionf q;
    q.x() = p.orientation.x;
    q.y() = p.orientation.y;
    q.z() = p.orientation.z;
    q.w() = p.orientation.w;
    iso.linear() = q.toRotationMatrix();
    return iso;
  }

};


// ------------------------------------------------------

// Main function
int main(int argc, char **argv){

  ros::init(argc,argv,"simulated_ideal_object_detector_node");
  ros::NodeHandle nh("~");

  IdealObjectDetectorNode mapper(nh);

  ros::spin();
  return 0;
}

