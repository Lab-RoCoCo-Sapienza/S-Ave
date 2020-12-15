#pragma once

#include "detection.h"
#include "model.h"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

typedef std::map<std::string,Eigen::Vector3i, std::less<std::string>,
Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3i> > > StringVector3iMap;


//this class implements an object detector in a simulation environment
class ObjectDetector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ObjectDetector();

    //setup model colors
    void setupModelColors();

    //for each model the 3d bounding box is transformed in the rgbd camera frame
    void setupDetections();

    // compute method
    void compute();

    //setters and getters
    inline void setCameraTransform(const Eigen::Isometry3f& camera_transform_){_camera_transform=camera_transform_;}
    inline void setModels(const ModelVector &models_){_models = models_;}
    inline const ModelVector &models() const {return _models;}
    inline void setInputCloud(const PointCloud::ConstPtr &cloud_){_cloud=cloud_;}
    inline const DetectionVector &detections() const {return _detections;}

    inline std::string& environment() {return _environment;}

  protected:

    //camera transform
    Eigen::Isometry3f _camera_transform;

    //camera_link to optical_frame transform
    Eigen::Isometry3f _camera_offset,_camera_offset_inv;

    //vector of models detected by the logical camera
    ModelVector _models;

    // input point cloud
    PointCloud::ConstPtr _cloud;

    //vector of detections
    DetectionVector _detections;

    //environment identifier
    std::string _environment;

    //model colors map
    StringVector3iMap _model_colors;
};

