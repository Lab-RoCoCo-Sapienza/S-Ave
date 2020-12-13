#include "detection.h"

Detection::Detection(){
  _size=0;
  _top_left = Eigen::Vector2i(10000,10000);
  _bottom_right = Eigen::Vector2i(-10000,-10000);
}

Detection::Detection(const std::string &type_,
                     const Eigen::Vector2i &top_left_,
                     const Eigen::Vector2i &bottom_right_,
                     const std::vector<Eigen::Vector2i> &pixels_,
                     const Eigen::Vector3i &color_):
  _type(type_),
  _top_left(top_left_),
  _bottom_right(bottom_right_),
  _pixels(pixels_),
  _color(color_),
  _size(0){}

void Detection::setup(const std::string &type, const Eigen::Vector3i& color){
  _type = type;
  _color = color;
  _size=0;
  _top_left = Eigen::Vector2i(10000,10000);
  _bottom_right = Eigen::Vector2i(-10000,-10000);

}
