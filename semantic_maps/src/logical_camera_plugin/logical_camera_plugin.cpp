#include "logical_camera_plugin.h"

using namespace gazebo;
using namespace std;
using namespace ros;

GZ_REGISTER_SENSOR_PLUGIN(LogicalCameraPlugin);

void LogicalCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
  // Get the parent sensor.
  this->parentSensor = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor){
    gzerr << "LogicalCameraPlugin requires a LogicalCameraSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&LogicalCameraPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  ROS_INFO("LogicalCameraPlugin correctly loaded!!!");
  ROS_INFO("_near:=%g",this->parentSensor->Near());
  ROS_INFO("_far:=%g",this->parentSensor->Far());
  ROS_INFO("_horizontalFOV:=%g",*(this->parentSensor->HorizontalFOV()));
  ROS_INFO("_aspect_ratio:=%g",this->parentSensor->AspectRatio());

  nh = new ros::NodeHandle("~");
  image_pub = nh->advertise<semantic_maps::LogicalImage>("logical_camera_image", 1, true);
}

void LogicalCameraPlugin::OnUpdate(){
  msgs::LogicalCameraImage logical_image;
  semantic_maps::LogicalImage msg;

  logical_image = this->parentSensor->Image();
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  if (!scene || !scene->Initialized())
    return;

//  msg.header.stamp = ros::Time::now();
  msg.header.stamp = ros::Time(this->parentSensor->LastUpdateTime().Double());
  msg.header.frame_id = "logical_camera_link";

  msg.pose.position.x = logical_image.pose().position().x();
  msg.pose.position.y = logical_image.pose().position().y();
  msg.pose.position.z = logical_image.pose().position().z();

  msg.pose.orientation.x = logical_image.pose().orientation().x();
  msg.pose.orientation.y = logical_image.pose().orientation().y();
  msg.pose.orientation.z = logical_image.pose().orientation().z();
  msg.pose.orientation.w = logical_image.pose().orientation().w();

  int number_of_models = logical_image.model_size();
  for(int i=0; i < number_of_models; i++){
    semantic_maps::Model model_msg;

    if (!logical_image.model(i).name().find("ignore") || !logical_image.model(i).name().find("phd_office_office_chair") )
      continue;

    if (logical_image.model(i).name() == "restaurant"||
        logical_image.model(i).name() == "test_apartment_2" ||
        logical_image.model(i).name() == "ground_plane" ||
        logical_image.model(i).name() == "apartment_1"||
        logical_image.model(i).name() == "apartment_2"||
        logical_image.model(i).name() == "apartment_3"||
        logical_image.model(i).name() == "apartment_4"||
        logical_image.model(i).name() == "grass_robocup"||
        logical_image.model(i).name() == "rococolab"||
        logical_image.model(i).name() == "phd_office"||
        logical_image.model(i).name() == "prof_office"||
        logical_image.model(i).name() == "orazio_world"||
        logical_image.model(i).name() == "")
      continue;


    rendering::VisualPtr visual = scene->GetVisual(logical_image.model(i).name());

    if (!visual)
      continue;

    ignition::math::Box bounding_box = visual->BoundingBox();

    model_msg.pose.position.x = logical_image.model(i).pose().position().x();
    model_msg.pose.position.y = logical_image.model(i).pose().position().y();
    model_msg.pose.position.z = logical_image.model(i).pose().position().z();

    model_msg.pose.orientation.x = logical_image.model(i).pose().orientation().x();
    model_msg.pose.orientation.y = logical_image.model(i).pose().orientation().y();
    model_msg.pose.orientation.z = logical_image.model(i).pose().orientation().z();
    model_msg.pose.orientation.w = logical_image.model(i).pose().orientation().w();

    model_msg.size.x = bounding_box.XLength();
    model_msg.size.y = bounding_box.YLength();
    model_msg.size.z = bounding_box.ZLength();

//    model_msg.min.x = bounding_box.GetCenter().x - bounding_box.GetSize().x/2.0;
//    model_msg.min.y = bounding_box.GetCenter().y - bounding_box.GetSize().y/2.0;
//    model_msg.min.z = bounding_box.GetCenter().z - bounding_box.GetSize().z/2.0;

//    model_msg.max.x = bounding_box.GetCenter().x + bounding_box.GetSize().x/2.0;
//    model_msg.max.y = bounding_box.GetCenter().y + bounding_box.GetSize().y/2.0;
//    model_msg.max.z = bounding_box.GetCenter().z + bounding_box.GetSize().z/2.0;


    model_msg.min.x = bounding_box.Min().X();
    model_msg.min.y = bounding_box.Min().Y();
    model_msg.min.z = bounding_box.Min().Z();

    model_msg.max.x = bounding_box.Max().X();
    model_msg.max.y = bounding_box.Max().Y();
    model_msg.max.z = bounding_box.Max().Z();

    model_msg.type = logical_image.model(i).name();

    msg.models.push_back(model_msg);
  }

  this->image_pub.publish(msg);
}
