#include "model.h"

Model::Model(const std::string &type_,
             const Eigen::Isometry3f &pose_,
             const Eigen::Vector3f &min_,
             const Eigen::Vector3f &max_):
  _type(type_),
  _pose(pose_),
  _min(min_),
  _max(max_){}

//bool Model::inRange(const Point &point){
//  return (point.x >= _min.x() && point.x <= _max.x() &&
//          point.y >= _min.y() && point.y <= _max.y() &&
//          point.z >= _min.z() && point.z <= _max.z());
//}

bool Model::inRange(const Point &point){
  return (point.x >= _min.x() && point.x <= _max.x() &&
          point.y >= _min.y() && point.y <= _max.y() &&
          point.z >= _min.z() && point.z <= _max.z());
}
