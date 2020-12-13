#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Detection;
typedef std::vector<Detection> DetectionVector;

//this class is a container for the output of an object detector
class Detection{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Detection();

    Detection(const std::string& type_,
              const Eigen::Vector2i& top_left_,
              const Eigen::Vector2i& bottom_right_,
              const std::vector<Eigen::Vector2i>& pixels_,
              const Eigen::Vector3i &color_);

    void setup(const std::string &type, const Eigen::Vector3i& color);

    //setters and getters
    inline const std::string &type() const {return _type;}
    inline std::string &type() {return _type;}
    inline const Eigen::Vector2i &topLeft() const {return _top_left;}
    inline Eigen::Vector2i &topLeft() {return _top_left;}
    inline const Eigen::Vector2i &bottomRight() const {return _bottom_right;}
    inline Eigen::Vector2i &bottomRight() {return _bottom_right;}
    inline const std::vector<Eigen::Vector2i> &pixels() const {return _pixels;}
    inline std::vector<Eigen::Vector2i>& pixels() {return _pixels;}
    inline const int size() const {return _size;}
    inline int &size() {return _size;}
    inline const Eigen::Vector3i &color() const {return _color;}
    inline Eigen::Vector3i &color() {return _color;}


  private:
    //semantic class of the detected object
    std::string _type;

    //top left pixel of the image bounding box
    Eigen::Vector2i _top_left;

    //bottom right pixel of the image bounding box
    Eigen::Vector2i _bottom_right;

    //array of pixels that belong to the detected object
    std::vector<Eigen::Vector2i> _pixels;

    //size of pixels array
    int _size;

    //class color (only for visualization)
    Eigen::Vector3i _color;

};

