#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <semantic_maps/LogicalImage.h>
#include <semantic_maps/Model.h>
#include <algorithm>
#include <vector>

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/point_cloud_conversion.h>




#include "semantic_maps/ncnn_config.h"
#ifdef GPU_SUPPORT
  #include "gpu.h"
  #include "semantic_maps/gpu_support.h"
#endif

/////////////////////////////////
#include <semantic_maps/ncnn_yolov5.h>
#include <semantic_maps/Object.h>
ncnnYoloV5 engine;
/////////////////////////////////

semantic_maps::Object objMsg;
//sensor_msgs::PointCloud cloud_msg;
ros::Subscriber map_sub;
ros::Subscriber depth_points_sub;
ros::Subscriber camera_tf_sub;
ros::Subscriber logical_img_sub;
ros::Publisher obj_pub;
std::vector<Object> objects;
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr DepthImage;
ros::Time last_time;
bool display_output;
double prob_threshold;
bool enable_gpu;


class YoloV5Focus : public ncnn::Layer
{
public:
    YoloV5Focus()
    {
        one_blob_only = true;
    }

    virtual int forward(const ncnn::Mat& bottom_blob, ncnn::Mat& top_blob, const ncnn::Option& opt) const
    {
        int w = bottom_blob.w;
        int h = bottom_blob.h;
        int channels = bottom_blob.c;

        int outw = w / 2;
        int outh = h / 2;
        int outc = channels * 4;

        top_blob.create(outw, outh, outc, 4u, 1, opt.blob_allocator);
        if (top_blob.empty())
            return -100;

        #pragma omp parallel for num_threads(opt.num_threads)
        for (int p = 0; p < outc; p++)
        {
            const float* ptr = bottom_blob.channel(p % channels).row((p / channels) % 2) + ((p / channels) / 2);
            float* outptr = top_blob.channel(p);

            for (int i = 0; i < outh; i++)
            {
                for (int j = 0; j < outw; j++)
                {
                    *outptr = *ptr;

                    outptr += 1;
                    ptr += 2;
                }

                ptr += w;
            }
        }

        return 0;
    }
};

DEFINE_LAYER_CREATOR(YoloV5Focus)


class Model
{
    private:
      std::string _type;
      Eigen::Isometry3f _pose;
      Eigen::Vector3f _min;
      Eigen::Vector3f _max;

    public:
      Model(const std::string &type_ = "",
          const Eigen::Isometry3f &pose_ = Eigen::Isometry3f::Identity(),
          const Eigen::Vector3f &min_ = Eigen::Vector3f::Zero(),
          const Eigen::Vector3f &max_ = Eigen::Vector3f::Zero()){}

      const std::string &type() const {return _type;}
      std::string &type() {return _type;}

      const Eigen::Isometry3f &pose() const {return _pose;}
      Eigen::Isometry3f &pose() {return _pose;}

      inline const Eigen::Vector3f &min() const {return _min;}
      inline Eigen::Vector3f &min() {return _min;}

      inline const Eigen::Vector3f &max() const {return _max;}
      inline Eigen::Vector3f &max() {return _max;}
};






void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Subscribed to 'map' topic");
}


void DepthPointsCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'camera/depth/points' topic");    
}

void CameraTFCallback(const geometry_msgs::Transform::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'camera_tf' topic");    
}

void LogicalCameraCallback(const semantic_maps::LogicalImage::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'logical_camera_image' topic");    
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg, int n_threads)
{
  try {
    //boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info_msg_ptr;
    //camera_info_msg_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo> ("camera/rgb/camera_info", ros::Duration (10));
    //boost::array<double, 9> K = camera_info_msg_ptr->K;
    //ROS_INFO("K = %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f", K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8]);

    //boost::shared_ptr<sensor_msgs::Image const> depth_image_msg_ptr;
    //depth_image_msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image> ("camera/depth/image_raw", ros::Duration (10));
    //DepthImage = cv_bridge::toCvCopy(depth_image_msg_ptr, sensor_msgs::image_encodings::TYPE_32FC1);

    //sensor_msgs::PointCloud2 cloud2_msg;
    //cloud2_msg = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points", ros::Duration(5.0));
    //sensor_msgs::convertPointCloud2ToPointCloud(cloud2_msg, cloud_msg);
    //ROS_INFO("Number of points: %d", cloud_msg.points.size());
    boost::shared_ptr<sensor_msgs::LaserScan const> laser_msg_ptr;
    laser_msg_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan> ("scan", ros::Duration (10));
    int range_size = laser_msg_ptr->ranges.size();
    ROS_INFO("Laser range: %d", range_size);
    
    auto minmax = std::minmax_element(std::begin(laser_msg_ptr->ranges), std::end(laser_msg_ptr->ranges));
    ROS_INFO("Smallest Distance robot-object: %.2f", *(minmax.first));

    // Gazebo
    boost::shared_ptr<semantic_maps::LogicalImage const> logical_image_msg_ptr;
    logical_image_msg_ptr = ros::topic::waitForMessage<semantic_maps::LogicalImage> ("gazebo/logical_camera_image", ros::Duration (10));
    int num_models = logical_image_msg_ptr->models.size();
    //std::vector<Model> models(num_models);

    for(int i=0; i < num_models; ++i){
      //semantic_maps::Model obj_model = logical_img_msg_ptr->models[i];
      //auto model_name = obj_model.type;
      //int bb_x = obj_model.size.x;
      //int bb_y = obj_model.size.y;
      //std::string obj_name = logical_image_msg_ptr->models[i].name;
      //ROS_INFO("obj_name type is: ", );
      float bb_width = (logical_image_msg_ptr->models[i].size.x) * 100.0f; // length in cm
      float bb_height = (logical_image_msg_ptr->models[i].size.y) * 100.0f; // length in cm
      float area_gaz = bb_width * bb_height;
      ROS_INFO("Model: %s = %.3f, %.3f, with area of %.3f", logical_image_msg_ptr->models[i].type.c_str(), bb_width, bb_height, area_gaz);
    }


    ros::Time current_time = ros::Time::now();
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //DepthImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    engine.detect(cv_ptr->image, objects, n_threads);

    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];
        if (obj.prob > prob_threshold)
        {
          ROS_INFO("%s \t = %.5f at %.2f %.2f %.2f x %.2f", class_names[obj.label], obj.prob,
          obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);
          objMsg.header.seq++;
          objMsg.header.stamp = current_time;
          objMsg.probability = obj.prob;
          objMsg.label = class_names[obj.label];
          objMsg.boundingbox.position.x = obj.rect.x;
          objMsg.boundingbox.position.y = obj.rect.y;
          objMsg.boundingbox.size.x = obj.rect.width;
          objMsg.boundingbox.size.y = obj.rect.height;
          obj_pub.publish(objMsg);
          float area = obj.rect.width * obj.rect.height;
          ROS_INFO("Area = %.2f ", area);

          /*float xmax = obj.rect.x + obj.rect.width;
          float xmin = obj.rect.x;
          float ymax = obj.rect.y + obj.rect.height;
          float ymin = obj.rect.y;

          float x_center = ((xmax-xmin)/2)+xmin;
          float y_center = ((ymax-ymin)/2)+ymin;
          int ind = (int)(x_center + (y_center*cloud2_msg.width));

          float X_real = (float)cloud_msg.points[ind].x;
          float Y_real = (float)cloud_msg.points[ind].y;
          float Z_real = (float)cloud_msg.points[ind].z;
          ROS_INFO("x_center= %.5f, y_center = %.5f, indice = %d, cloud_width = %d, Z reale = %.5f", x_center, y_center, ind, cloud2_msg.width, Z_real);
          */

          /*float xmax = obj.rect.x + obj.rect.width;
          float xmin = obj.rect.x;
          float ymax = obj.rect.y + obj.rect.height;
          float ymin = obj.rect.y;

          float xc = ((xmax-xmin)/2)+xmin;
          float yc = ((ymax-ymin)/2)+ymin;

          // ---------------------------
          ROS_INFO("------------------- BOTTOM LEFT -------------");
          float x_img = xc - (obj.rect.width/2);
          float y_img = yc + (obj.rect.height/2);
          int ind = (int)((x_img) + (y_img)*cloud2_msg.width);
          ROS_INFO("x_img= %.5f, y_img = %.5f, indice = %d, cloud_width = %d", x_img, y_img, ind, cloud2_msg.width);


          float X_real = (float)cloud_msg.points[ind].x;
          float Y_real = (float)cloud_msg.points[ind].y;
          float Z_real = (float)cloud_msg.points[ind].z;
          ROS_INFO("BOTTOM LEFT: Real X %.5f, Real Y %.5f, Real Z %.5f", X_real, Y_real, Z_real);
          
          // ---------------------------
          ROS_INFO("------------------- BOTTOM RIGHT -------------");
          x_img = xc + (obj.rect.width/2);
          y_img = yc + (obj.rect.height/2);
          ind = (int)((x_img) + (y_img)*cloud2_msg.width);
          ROS_INFO("x_img= %.5f, y_img = %.5f, indice = %d, cloud_width = %d", x_img, y_img, ind, cloud2_msg.width);


          X_real = (float)cloud_msg.points[ind].x;
          Y_real = (float)cloud_msg.points[ind].y;
          Z_real = (float)cloud_msg.points[ind].z;
          ROS_INFO("BOTTOM RIGHT: Real X %.5f, Real Y %.5f, Real Z %.5f", X_real, Y_real, Z_real);
          */
          

        }
    }

    if (display_output) {
      engine.draw(cv_ptr->image, objects, (current_time-last_time).toSec());
    }
    last_time = current_time;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("CV bridge exception: %s", e.what());
    return;
  }
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolov5_object_detector_node"); /**/
  ros::NodeHandle nhLocal("~");
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getName();
  int gpu_device;
  nhLocal.param("gpu_device", gpu_device, 0);
  nhLocal.param("enable_gpu", enable_gpu, false);

#ifndef GPU_SUPPORT
  ROS_WARN_STREAM(node_name << " running on CPU");
#endif
#ifdef GPU_SUPPORT
  ROS_INFO_STREAM(node_name << " with GPU_SUPPORT, selected gpu_device: " << gpu_device);
  g_vkdev = ncnn::get_gpu_device(selectGPU(gpu_device));
  g_blob_vkallocator = new ncnn::VkBlobAllocator(g_vkdev);
  g_staging_vkallocator = new ncnn::VkStagingAllocator(g_vkdev);
  engine.neuralnet.opt.use_vulkan_compute = enable_gpu;
  engine.neuralnet.set_vulkan_device(g_vkdev);
#endif

  const std::string package_name = "semantic_maps";
  std::string path = ros::package::getPath(package_name)+("/assets/ncnn_models/models/");
  ROS_INFO("Assets path: %s", path.c_str());

  std::string model_file, param_file;
  engine.neuralnet.register_custom_layer("YoloV5Focus", YoloV5Focus_layer_creator);

  nhLocal.param("model_file", model_file, std::string("yolov5l-opt.bin"));
  nhLocal.param("param_file", param_file, std::string("yolov5l-opt.param"));
  engine.neuralnet.load_param((path+param_file).c_str());
  engine.neuralnet.load_model((path+model_file).c_str());
  ROS_INFO("Loaded: %s", model_file.c_str());

  int num_threads;
  nhLocal.param("num_threads", num_threads, ncnn::get_cpu_count());
  nhLocal.param("display_output", display_output, true);

  map_sub = n.subscribe("/map", 1, MapCallback);
  depth_points_sub = n.subscribe("/camera/depth/points", 1, DepthPointsCallback);
  camera_tf_sub = n.subscribe("/camera_tf", 1, CameraTFCallback);
  logical_img_sub = n.subscribe("/gazebo/logical_camera_image", 1, LogicalCameraCallback);
  obj_pub = n.advertise<semantic_maps::Object>("/detected_objects", 50);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber video = it.subscribe("/camera/rgb/image_raw", 1, boost::bind(&imageCallback, _1, num_threads));
  //image_transport::Subscriber depth = it.subscribe("/camera/depth/image_raw", 1, boost::bind(&depthImageCallback, _1, num_threads));


#ifdef GPU_SUPPORT
  ncnn::create_gpu_instance();
#endif
  while (ros::ok()) {
    ros::spinOnce();
  }
#ifdef GPU_SUPPORT
  ncnn::destroy_gpu_instance();
#endif

  ROS_INFO("Finished");


  return 0;
}
