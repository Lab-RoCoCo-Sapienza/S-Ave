#pragma once

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <object_detector/detection.h>

#include "object.h"

class SemanticMapper{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SemanticMapper();

    virtual ~SemanticMapper();

    //set robot pose
    inline void setGlobalT(const Eigen::Isometry3f &globalT_){_globalT = globalT_;}

    //specialized extractObjects method
    void extractObjects(const DetectionVector &detections,
                        const PointCloud::ConstPtr &points);

    //specialized findAssociations method
    void findAssociations();

    //specialized mergeMaps method
    void mergeMaps();

    const ObjectPtrVector* globalMap() const {return _global_map;}
    const ObjectPtrVector* localMap() const {return _local_map;}

    const ObjectPtrIdMap& associations() const {return _associations;}

  protected:

    //pose of the robot w.r.t. the global map
    Eigen::Isometry3f _globalT;

    Eigen::Isometry3f _camera_offset;

    //flags
    bool _local_set;
    bool _global_set;

    //map built from the current frame
    ObjectPtrVector *_local_map;

    //actual map that stores objects in a global reference frame and gets updated for each new observation
    ObjectPtrVector *_global_map;

    //this map stores the output of the data-association
    ObjectPtrIdMap _associations;
};
