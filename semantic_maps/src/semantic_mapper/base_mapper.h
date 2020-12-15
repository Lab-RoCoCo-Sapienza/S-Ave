#pragma once

#include "semantic_map.h"
#include "perception/detection.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace lucrezio_spme{
  //abstract class for semantic mapper
  class BaseMapper{
  public:
    BaseMapper();

    virtual ~BaseMapper();

    //this function takes as input the outcome of the object detector and builds a local map
    virtual void extractObjects(const DetectionVector &detections,
                                const cv::Mat &depth_image_) = 0;

    //this function finds correspondences between objects in the local and the global map
    virtual void findAssociations() = 0;

    //this function updates the objects in the global map with the newly observed ones
    virtual void mergeMaps() = 0;

  protected:

    //map built from the current frame
    SemanticMap *_local_map;

    //actual map that stores objects in a global reference frame and gets updated for each new observation
    SemanticMap *_global_map;

    //this map stores the output of the data-association
    ObjectPtrIdMap _associations;

  };

}


