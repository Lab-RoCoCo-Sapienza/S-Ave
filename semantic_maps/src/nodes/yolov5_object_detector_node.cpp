#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>

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
ros::Subscriber map_sub;
ros::Subscriber depth_points_sub;
ros::Subscriber camera_tf_sub;
ros::Publisher obj_pub;
std::vector<Object> objects;
cv_bridge::CvImagePtr cv_ptr;
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



void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Subscribed to 'map' topic");
}


void DepthPointsCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'camera/depth/points' topic");    
}

void CameraTFCallback(const geometry_msgs::Transform::ConstPtr &msg) {
    ROS_INFO("Subscribed to 'camera_tf' topic");    
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg, int n_threads)
{
  try {
    ros::Time current_time = ros::Time::now();
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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
  obj_pub = n.advertise<semantic_maps::Object>("/detected_objects", 50);

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
