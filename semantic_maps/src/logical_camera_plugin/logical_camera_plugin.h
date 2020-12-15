#pragma once

#include <string>
#include <sstream>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Box.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <semantic_maps/LogicalImage.h>

namespace gazebo {
class LogicalCameraPlugin : public SensorPlugin{
public:
    LogicalCameraPlugin() : SensorPlugin(){}
    virtual ~LogicalCameraPlugin() {}

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
protected:
    ros::NodeHandle *nh;
    ros::Publisher image_pub;
private:
    virtual void OnUpdate();

    sensors::LogicalCameraSensorPtr parentSensor;
    event::ConnectionPtr updateConnection;
};
}
