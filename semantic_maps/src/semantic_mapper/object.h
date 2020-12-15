#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/norms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ros/time.h>
#include <octomap/OcTree.h>

#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Object;
typedef Object* ObjectPtr;
typedef std::vector<ObjectPtr> ObjectPtrVector;
typedef std::map<ObjectPtr,int> ObjectPtrIdMap;
typedef std::set<ObjectPtr> ObjectPtrSet;
typedef std::map<std::string,Object> ObjectStringMap;
typedef std::map<std::string,ObjectPtr> StringObjectPtrMap;

class GtObject;
typedef std::map<std::string,GtObject> GtObjectStringMap;


//this class is a container for a 3d object that composes the semantic map
class Object {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ctors
    Object();

    Object(const std::string &model_,
           const Eigen::Vector3f &position_,
           const Eigen::Vector3f &min_=Eigen::Vector3f::Zero(),
           const Eigen::Vector3f &max_=Eigen::Vector3f::Zero(),
           const Eigen::Vector3f &color_=Eigen::Vector3f::Zero(),
           const PointCloud::Ptr &cloud_=0);

    Object(const std::string &model_,
           const Eigen::Vector3f &position_,
           const Eigen::Vector3f &min_,
           const Eigen::Vector3f &max_,
           const Eigen::Vector3f &color_,
           const std::string &cloud_filename,
           const std::string &octree_filename,
           const std::string &fre_voxel_cloud_filename,
           const std::string &occ_voxel_cloud_filename);
    
    Object(const std::string &model_,    // AHHHHHHHHHHH
               const Eigen::Vector3f &position_,
               const Eigen::Vector3f &min_,
               const Eigen::Vector3f &max_,
               const Eigen::Vector3f &color_,
               const PointCloud::Ptr & cloud_,
               octomap::OcTree* &octree_,
               const PointCloud::Ptr & fre_voxel_cloud_,
               const PointCloud::Ptr & occ_voxel_cloud_,
               const float _ocupancy_volume);

    Object(const Object& obj);

    //dtor
    ~Object();

    bool operator < (const Object &o) const;
    bool operator == (const Object &o) const;

    //setters and getters
    inline const std::string& model() const {return _model;}
    inline std::string& model() {return _model;}
    inline const Eigen::Vector3f& position() const {return _position;}
    inline Eigen::Vector3f& position() {return _position;}
    inline const Eigen::Vector3f& min() const {return _min;}
    inline Eigen::Vector3f& min() {return _min;}
    inline const Eigen::Vector3f& max() const {return _max;}
    inline Eigen::Vector3f& max() {return _max;}
    inline const Eigen::Vector3f &color() const {return _color;}
    inline Eigen::Vector3f &color() {return _color;}
    inline const PointCloud::Ptr &cloud() const {return _cloud;}
    inline PointCloud::Ptr &cloud() {return _cloud;}
    inline const float ocupancy_volume() const {return _ocupancy_volume;}
    inline float ocupancy_volume() {return _ocupancy_volume;}    

    inline const PointCloud::Ptr &freVoxelCloud() const {return _fre_voxel_cloud;}
    inline const PointCloud::Ptr &occVoxelCloud() const {return _occ_voxel_cloud;}

    inline octomap::OcTree* octree() const {return _octree;}

    //check if a point falls in the bounding box
    bool inRange(const Point &point) const;

    //check if a point falls in the bounding box
    bool inRange(const float& x, const float& y, const float& z, const float& off) const;

    //merge two objects
    void merge(const ObjectPtr &o);

    //compute occupancy
    void updateOccupancy(const Eigen::Isometry3f& T, const PointCloud::Ptr &cloud);

  private:

    //name
    std::string _model;

    //position
    Eigen::Vector3f _position;

    //lower vertex of the object bounding box
    Eigen::Vector3f _min;

    //upper vertex of the object bounding box
    Eigen::Vector3f _max;

    //object color (for visualization only)
    Eigen::Vector3f _color;

    //object point cloud
    PointCloud::Ptr _cloud;

    //ocupancy volume
    float _ocupancy_volume;

    //last processed view
    octomap::point3d _last_processed_view;
    
    pcl::VoxelGrid<Point> _voxelizer;

    octomap::OcTree* _octree;
    PointCloud::Ptr _occ_voxel_cloud;
    PointCloud::Ptr _fre_voxel_cloud;
};

class GtObject{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  GtObject(const std::string& model_="",
        const Eigen::Vector3f& position_ = Eigen::Vector3f::Zero(),
        const Eigen::Vector3f& orientation_ = Eigen::Vector3f::Zero(),
           const Eigen::Vector3f& min_ = Eigen::Vector3f::Zero(),
           const Eigen::Vector3f& max_ = Eigen::Vector3f::Zero()):
    _model(model_),
    _position(position_),
    _orientation(orientation_),
    _min(min_),
    _max(max_)
  {}
  inline const std::string& model() const {return _model;}
  inline std::string& model() {return _model;}
  inline const Eigen::Vector3f& position() const {return _position;}
  inline Eigen::Vector3f& position() {return _position;}
  inline const Eigen::Vector3f& orientation() const {return _orientation;}
  inline Eigen::Vector3f& orientation() {return _orientation;}
  inline const Eigen::Vector3f& min() const {return _min;}
  inline Eigen::Vector3f& min() {return _min;}
  inline const Eigen::Vector3f& max() const {return _max;}
  inline Eigen::Vector3f& max() {return _max;}
protected:
  std::string _model;
  Eigen::Vector3f _position;
  Eigen::Vector3f _orientation;
  Eigen::Vector3f _min;
  Eigen::Vector3f _max;
};