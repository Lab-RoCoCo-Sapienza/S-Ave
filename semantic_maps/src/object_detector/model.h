#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Model;
typedef std::vector<Model> ModelVector;

class Model{
  public:
    Model(const std::string &type_ = "",
          const Eigen::Isometry3f &pose_ = Eigen::Isometry3f::Identity(),
          const Eigen::Vector3f &min_ = Eigen::Vector3f::Zero(),
          const Eigen::Vector3f &max_ = Eigen::Vector3f::Zero());

    const std::string &type() const {return _type;}
    std::string &type() {return _type;}

    const Eigen::Isometry3f &pose() const {return _pose;}
    Eigen::Isometry3f &pose() {return _pose;}

    inline const Eigen::Vector3f &min() const {return _min;}
    inline Eigen::Vector3f &min() {return _min;}

    inline const Eigen::Vector3f &max() const {return _max;}
    inline Eigen::Vector3f &max() {return _max;}

    //check if a point falls in the bounding box
    bool inRange(const Point &point);

  private:
    std::string _type;
    Eigen::Isometry3f _pose;
    Eigen::Vector3f _min;
    Eigen::Vector3f _max;
};

