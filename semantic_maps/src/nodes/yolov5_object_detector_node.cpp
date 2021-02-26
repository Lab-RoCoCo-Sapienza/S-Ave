#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <semantic_maps/LogicalImage.h>
#include <semantic_maps/ObjectArray.h>
#include <semantic_maps/Object.h>
#include <semantic_maps/Model.h>
#include <algorithm>
#include <vector>
//#include <tuple> // for tuple 

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>



#include "semantic_maps/ncnn_config.h"
#ifdef GPU_SUPPORT
  #include "gpu.h"
  #include "semantic_maps/gpu_support.h"
#endif

/////////////////////////////////
#include <semantic_maps/ncnn_yolov5.h>
ncnnYoloV5 engine;
/////////////////////////////////

semantic_maps::Object objMsg;
//sensor_msgs::PointCloud cloud_msg;
ros::Subscriber map_sub;
ros::Subscriber depth_points_sub;
ros::Subscriber camera_tf_sub;
ros::Subscriber logical_img_sub;
ros::Subscriber cmd_vel_sub;
//ros::Publisher obj_pub;
ros::Publisher list_obj_pub;
std::vector<Object> objects;
semantic_maps::ObjectArray list_obj_msg;
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr DepthImage;
ros::Time last_time;
bool display_output;
double prob_threshold;
bool enable_gpu;

std::string prev_label = "";
bool stationary_robot = false;
bool moved = true;
bool obj_multiple_detection;
int detected_objects = 0;



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



void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Subscribed to 'map' topic");
}


void DepthPointsCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'camera/depth/points' topic");    
}

void CameraTFCallback(const geometry_msgs::Transform::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'camera_tf' topic");    
}

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'lucrezio/cmd_vel' topic");    
}




void imageCallback(const sensor_msgs::ImageConstPtr& msg, int n_threads)
{
  try {
    
    boost::shared_ptr<sensor_msgs::LaserScan const> laser_msg_ptr;
    laser_msg_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan> ("scan", ros::Duration (10));
    int range_size = laser_msg_ptr->ranges.size();
    //ROS_INFO("Laser range: %d", range_size);
    
    auto minmax = std::minmax_element(std::begin(laser_msg_ptr->ranges), std::end(laser_msg_ptr->ranges));
    float distance = *(minmax.first) * 1000.0f;
    ROS_INFO("Smallest Distance robot-object: %.3f", distance);

    // Gazebo
    boost::shared_ptr<semantic_maps::LogicalImage const> logical_image_msg_ptr;
    logical_image_msg_ptr = ros::topic::waitForMessage<semantic_maps::LogicalImage> ("gazebo/logical_camera_image", ros::Duration (10));
    int num_models = logical_image_msg_ptr->models.size();

    //tuple <std::string, float, float> gaz_model;
    /*for(int i=0; i < num_models; ++i){

      float bb_width = (logical_image_msg_ptr->models[i].size.x) * 1000.0f; // length in mm
      float bb_height = (logical_image_msg_ptr->models[i].size.z) * 1000.0f; // length in mm
      //float area_gaz = bb_width * bb_height;
      std::string model_name = logical_image_msg_ptr->models[i].type;
      std::string first_part = model_name.substr(0, model_name.find(delimiter)); 
      //gaz_model = make_tuple(first_part, bb_width, bb_height);
      //vec.push_back(gaz_model);
      ROS_INFO("Gazebo Model: %s = %.3f, %.3f, with area of %.3f", first_part.c_str(), bb_width, bb_height, area_gaz);
    }*/

    boost::shared_ptr<geometry_msgs::Twist const> cmd_vel_msg_ptr;
    cmd_vel_msg_ptr = ros::topic::waitForMessage<geometry_msgs::Twist> ("lucrezio/cmd_vel", ros::Duration (15));


    ros::Time current_time = ros::Time::now();
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    engine.detect(cv_ptr->image, objects, n_threads);

    obj_multiple_detection = false;
    if (objects.size() != 0 && detected_objects != 0){
      /*if (cmd_vel_msg_ptr != NULL && cmd_vel_msg_ptr->linear.x != 0.0f){
        ROS_INFO("Robot linear");
        stationary_robot = true;
        moved = false;
      } else if (cmd_vel_msg_ptr != NULL && (cmd_vel_msg_ptr->angular.x != 0.0f || cmd_vel_msg_ptr->angular.y != 0.0f || cmd_vel_msg_ptr->angular.z != 0.0f) ) {
        ROS_INFO("Robot angular");
        detected_objects ++;
        stationary_robot = false;
        moved = true;
      } else*/ 
      if (cmd_vel_msg_ptr == NULL && !moved) {
        ROS_INFO("Robot stopped");
        stationary_robot = true;
      } else if (cmd_vel_msg_ptr == NULL && moved){
        ROS_INFO("Robot moved");
        detected_objects ++;
        stationary_robot = false;
        moved = false;
      } else {
        detected_objects ++;
        stationary_robot = false;
        moved = true;
      }
    } else if (objects.size() != 0 && detected_objects == 0) {
      ROS_INFO("Robot moved");
      detected_objects ++;
      stationary_robot = false;
      moved = false;
    }

    prob_threshold = engine.getThreshold();
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];
        if (obj.prob > prob_threshold)
        {
          if (!stationary_robot) {
            ROS_INFO("%s \t = %.5f at %.2f %.2f %.2f x %.2f", class_names[obj.label], obj.prob,
            obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);
            objMsg.header.seq++;
            objMsg.header.stamp = current_time;
            objMsg.label = class_names[obj.label];
            objMsg.probability = obj.prob;
            objMsg.distance = distance;
      
            objMsg.wyolo = obj.rect.width;
            objMsg.hyolo = obj.rect.height;
            objMsg.count = detected_objects;

            std::string final_name;
            for(int i=0; i < num_models; ++i) {
              std::string model_name = logical_image_msg_ptr->models[i].type;
              
              if (model_name.find("desk") || model_name.find("table")) {
                final_name = "dining table";
              } else if (model_name.find("monitor_pc") || model_name.find("tv")) {
                final_name = "tv";
              } else if (model_name.find("chair")) {
                final_name = "chair";
              } else if (model_name.find("pc")) {
                final_name = "laptop";
              } else if (model_name.find("refrigerator")) {
                final_name = "refrigerator";
              } else if (model_name.find("sofa")) {
                final_name = "couch";
              } else if (model_name.find("books")) {
                final_name = "book";
              } else if (model_name.find("bed")) {
                final_name = "bed";
              } else if (model_name.find("sink")) {
                final_name = "sink";
              } else if (model_name.find("stove")) {
                final_name = "oven";
              } else if (model_name.find("toilet")) {
                final_name = "toilet";
              } else if (model_name.find("microwave")) {
                final_name = "microwave";
              }

              
              if (final_name.c_str() == objMsg.label) {
                objMsg.wgazebo = (logical_image_msg_ptr->models[i].size.x) * 1000.0f; // length in mm
                objMsg.hgazebo = (logical_image_msg_ptr->models[i].size.z) * 1000.0f; // length in mm
                ROS_INFO("Gazebo Model: %s = %.3f, %.3f", final_name.c_str(), objMsg.wgazebo, objMsg.hgazebo);
                list_obj_msg.objects.push_back(objMsg);
                //obj_pub.publish(objMsg);
              }
            }
          }

            //float area = obj.rect.width * obj.rect.height;
            //ROS_INFO("Yolo Area = %.2f ", area);
          //}

          if (prev_label == ""){
            prev_label = objMsg.label;
            //ROS_INFO("Changed %s", prev_label.c_str());
          } else if (prev_label == objMsg.label && stationary_robot){
            obj_multiple_detection = true;
            ROS_INFO("Object has already been detected !!");
          }          
        }
    }
    list_obj_pub.publish(list_obj_msg);



    /*for (size_t i = 0; i < objects.size(); i++)
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
          ROS_INFO("Yolo Area = %.2f ", area);

          if (prev_label == ""){
            prev_label = objMsg.label;
            //ROS_INFO("Changed %s", prev_label.c_str());
          } else if (prev_label == objMsg.label && stationary_robot){
            obj_multiple_detection = true;
            ROS_INFO("Object has already been detected !!");
          }          
        }
    }*/
    ROS_INFO("Number of detected objects: %d", detected_objects);

    if (display_output && !obj_multiple_detection) {
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
  cmd_vel_sub = n.subscribe("/lucrezio/cmd_vel", 1, CmdVelCallback);
  //obj_pub = n.advertise<semantic_maps::Object>("/detected_objects", 50);
  list_obj_pub = n.advertise<semantic_maps::ObjectArray>("/detected_objects", 50);


  image_transport::ImageTransport it(n);
  image_transport::Subscriber video = it.subscribe("/camera/rgb/image_raw", 1, boost::bind(&imageCallback, _1, num_threads));


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
