#include "semantic_mapper.h"

SemanticMapper::SemanticMapper(){

  _local_map = new ObjectPtrVector();
  _global_map = new ObjectPtrVector();

  _associations.clear();

  _local_set = false;
  _global_set = false;

  _globalT.setIdentity();

  _camera_offset.setIdentity();
  _camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();
}

SemanticMapper::~SemanticMapper(){
  delete _local_map;
  delete _global_map;
}

void SemanticMapper::extractObjects(const DetectionVector &detections,
                                    const PointCloud::ConstPtr & points){

  //the first message populates the global map, the others populate the local map
  bool populate_global = false;
  if(!_global_set){
    populate_global = true;
    _global_set = true;
  } else {
    _local_map->clear();
    _local_set = true;
  }

  size_t w=points->width;

  for(const Detection& detection : detections){

    if(detection.pixels().size() < 10)
      continue;

    std::string model = detection.type();
    Eigen::Vector3f color = detection.color().cast<float>()/255.0f;

    const std::vector<Eigen::Vector2i> &pixels = detection.pixels();
    int num_pixels = pixels.size();
    PointCloud::Ptr cloud (new PointCloud());
    cloud->resize(num_pixels);
    int k=0;

    Eigen::Vector3f min(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    Eigen::Vector3f max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
    Eigen::Vector3f position = Eigen::Vector3f::Zero();

    for(int i=0; i<num_pixels; ++i){

      Point point = points->at(pixels[i].y(),pixels[i].x());

      if(std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z) < 1e-3 || point.z <= 0.1)
        continue;

      point = pcl::transformPoint(point,_globalT*_camera_offset);

      point.r = color.z()*255;
      point.g = color.y()*255;
      point.b = color.x()*255;

      cloud->at(k) = point;
      k++;
      if(point.x < min.x())
        min.x() = point.x;
      if(point.x > max.x())
        max.x() = point.x;
      if(point.y < min.y())
        min.y() = point.y;
      if(point.y > max.y())
        max.y() = point.y;
      if(point.z < min.z())
        min.z() = point.z;
      if(point.z > max.z())
        max.z() = point.z;
    }

    //check if object is empty
    if(!k)
      continue;

    cloud->resize(k);
    position = (min+max)/2.0f;

    ObjectPtr obj_ptr (new Object(model,position,min,max,color,cloud));
    obj_ptr->updateOccupancy(_globalT,cloud);

    if(populate_global)
      _global_map->push_back(obj_ptr);
    else
      _local_map->push_back(obj_ptr);

  }
}

void SemanticMapper::findAssociations(){
  if(!_global_set || !_local_set)
    return;

  const int local_size = _local_map->size();
  const int global_size = _global_map->size();

  _associations.clear();

  for(int i=0; i < global_size; ++i){
    const ObjectPtr &global = (*_global_map)[i];
    const std::string &global_model = global->model();

    ObjectPtr local_best = nullptr;
    float best_error = std::numeric_limits<float>::max();

    for(int j=0; j < local_size; ++j){
      const ObjectPtr &local = (*_local_map)[j];
      const std::string &local_model = local->model();

      if(local_model != global_model)
        continue;

      Eigen::Vector3f e_c = local->position() - global->position();

      float error = e_c.transpose()*e_c;

      if(error<best_error){
        best_error = error;
        local_best = local;
      }
    }

    if(!local_best)
      continue;
    _associations[local_best] = i;
  }
}

void SemanticMapper::mergeMaps(){
  if(!_global_set || !_local_set)
    return;

  int added = 0, merged = 0;

  for(int i=0; i<_local_map->size(); ++i){
    const ObjectPtr &local = (*_local_map)[i];
    ObjectPtrIdMap::iterator it = _associations.find(local);
    int association_id = -1;
    if(it != _associations.end()){
      association_id = it->second;
      ObjectPtr &global_associated = (*_global_map)[association_id];

      if(local->model() != global_associated->model())
        continue;

      global_associated->updateOccupancy(_globalT,local->cloud());
      global_associated->merge(local);
      merged++;
    } else {
      _global_map->push_back(local);
      added++;
    }
  }
}
