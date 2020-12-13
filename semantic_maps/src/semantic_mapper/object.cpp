#include "object.h"

namespace YAML {
  template <typename Scalar, int Rows, int Cols>
  struct convert<Eigen::Matrix<Scalar,Rows,Cols> > {
    static Node encode(const Eigen::Matrix<Scalar,Rows,Cols> &mat){
      static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,"Static matrices only");
      Node node(NodeType::Sequence);
      for (int i = 0; i < Rows; i++)
        for (int j = 0; j < Cols; j++)
          node[i*Cols + j] = static_cast<double>(mat(i, j));
      return node;
    }

    static bool decode(const Node &node, Eigen::Matrix<Scalar,Rows,Cols> &mat){
      static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,"Static matrices only");
      if (!node.IsSequence() || node.size() != Rows*Cols)
        return false;
      for (int i = 0; i < Rows; i++) {
        for (int j = 0; j < Cols; j++) {
          mat(i, j) = static_cast<Scalar>(node[i * Cols + j].as<double>());
        }
      }
      return true;
    }
  };

  template<>
  struct convert<GtObject> {
    static Node encode(const GtObject &obj) {
      Node node;
      const std::string& model = obj.model();
      node["model"] = model;
      node["position"] = obj.position();
      node["orientation"] = obj.orientation();
      return node;
    }

    static bool decode(const Node& node, GtObject &obj) {
      if(!node.IsMap())
        return false;
      obj.model() = node["model"].as<std::string>();
      //      obj.position() = node["position"].as<Eigen::Vector3f>();
      obj.orientation() = node["orientation"].as<Eigen::Vector3f>();
      Eigen::Vector3f min = node["min"].as<Eigen::Vector3f>();
      Eigen::Vector3f max = node["max"].as<Eigen::Vector3f>();
      obj.min() = min;
      obj.max() = max;
      obj.position() = (min+max)/2.0f;
      return true;
    }
  };

  template<>
  struct convert<Object> {
    static Node encode(const Object &obj) {
      Node node;
      const std::string& model = obj.model();
      node["model"] = model;
      node["position"] = obj.position();
      node["min"] = obj.min();
      node["max"] = obj.max();
      const std::string cloud_filename = model+".pcd";
      node["cloud"] = cloud_filename;
      pcl::io::savePCDFileASCII(cloud_filename,*(obj.cloud()));
      return node;
    }

    static bool decode(const Node& node, Object &obj) {
      if(!node.IsMap())
        return false;
      obj.model() = node["model"].as<std::string>();
      obj.position() = node["position"].as<Eigen::Vector3f>();
      obj.min() = node["min"].as<Eigen::Vector3f>();
      obj.max() = node["max"].as<Eigen::Vector3f>();
      const std::string cloud_filename = node["cloud"].as<std::string>();
      pcl::io::loadPCDFile<Point> (cloud_filename, *(obj.cloud()));
      return true;
    }
  };
}

using namespace std;

Object::Object():_octree(new octomap::OcTree(0.05)){ //0.05
  _model = "";
  _position.setZero();
  _min.setZero();
  _max.setZero();
  _color.setZero();
  _cloud = PointCloud::Ptr (new PointCloud());
  _fre_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _occ_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _ocupancy_volume = 0.0;
}

Object::Object(const string &model_,
               const Eigen::Vector3f &position_,
               const Eigen::Vector3f &min_,
               const Eigen::Vector3f &max_,
               const Eigen::Vector3f &color_,
               const PointCloud::Ptr & cloud_):
  _model(model_),
  _position(position_),
  _min(min_),
  _max(max_),
  _color(color_),
  _cloud(cloud_),
  _octree(new octomap::OcTree(0.05)),
  _fre_voxel_cloud(new PointCloud()),
  _occ_voxel_cloud(new PointCloud()){_ocupancy_volume= 0.0;}

Object::Object(const string &model_,
               const Eigen::Vector3f &position_,
               const Eigen::Vector3f &min_,
               const Eigen::Vector3f &max_,
               const Eigen::Vector3f &color_,
               const string &cloud_filename,
               const string &octree_filename,
               const string &fre_voxel_cloud_filename,
               const string &occ_voxel_cloud_filename):
  _model(model_),
  _position(position_),
  _min(min_),
  _max(max_),
  _color(color_),
  _cloud(new PointCloud()),
  _fre_voxel_cloud(new PointCloud()),
  _occ_voxel_cloud(new PointCloud()){
    
  _ocupancy_volume = 0.0;

  pcl::io::loadPCDFile<Point> (cloud_filename, *_cloud);

  _octree = new octomap::OcTree(octree_filename);

  if(fre_voxel_cloud_filename != "...")
    pcl::io::loadPCDFile<Point> (fre_voxel_cloud_filename, *_fre_voxel_cloud);

  if(occ_voxel_cloud_filename != "...")
    pcl::io::loadPCDFile<Point> (occ_voxel_cloud_filename, *_occ_voxel_cloud);

}

Object::Object(const Object &obj):
  _model(obj.model()),
  _position(obj.position()),
  _min(obj.min()),
  _max(obj.max()),
  _color(obj.color()),
  _cloud(obj.cloud()),
  _octree(obj.octree()),
  _fre_voxel_cloud(obj.freVoxelCloud()),
  _occ_voxel_cloud(obj.occVoxelCloud()),
  _ocupancy_volume(obj.ocupancy_volume()){}


Object::Object(const string &model_, 
               const Eigen::Vector3f &position_,
               const Eigen::Vector3f &min_,
               const Eigen::Vector3f &max_,
               const Eigen::Vector3f &color_,
               const PointCloud::Ptr & cloud_,
               octomap::OcTree* &octree_,
               const PointCloud::Ptr & fre_voxel_cloud_,
               const PointCloud::Ptr & occ_voxel_cloud_,
               const float ocupancy_volume_):
  _model(model_),
  _position(position_),
  _min(min_),
  _max(max_),
  _color(color_),
  _cloud(cloud_),
  _octree(octree_),
  _fre_voxel_cloud(fre_voxel_cloud_),
  _occ_voxel_cloud(occ_voxel_cloud_),
  _ocupancy_volume(ocupancy_volume_){}

Object::~Object(){
  delete _octree;
}

bool Object::operator <(const Object &o) const{
  return (_model.compare(o.model()) < 0);
}

bool Object::operator ==(const Object &o) const{
  return (_model.compare(o.model()) == 0);
}

bool Object::inRange(const Point &point) const{
  return (point.x >= _min.x() && point.x <= _max.x() &&
          point.y >= _min.y() && point.y <= _max.y() &&
          point.z >= _min.z() && point.z <= _max.z());
}

bool Object::inRange(const float &x, const float &y, const float &z, const float &off) const{
  return (x >= _min.x() - off && x <= _max.x() + off &&
          y >= _min.y() - off && y <= _max.y() + off &&
          z >= _min.z() - off && z <= _max.z() + off);
}

void Object::merge(const ObjectPtr & o){
  if(o->min().x() < _min.x())
    _min.x() = o->min().x();
  if(o->max().x() > _max.x())
    _max.x() = o->max().x();
  if(o->min().y() < _min.y())
    _min.y() = o->min().y();
  if(o->max().y() > _max.y())
    _max.y() = o->max().y();
  if(o->min().z() < _min.z())
    _min.z() = o->min().z();
  if(o->max().z() > _max.z())
    _max.z() = o->max().z();

  _position = (_min+_max)/2.0f;
  
  //add new points
  *_cloud += *o->cloud();

  //voxelize
  PointCloud::Ptr cloud_filtered (new PointCloud());
  _voxelizer.setInputCloud(_cloud);
  //  _voxelizer.setLeafSize(0.05f,0.05f,0.05f);
  _voxelizer.setLeafSize(0.02f,0.02f,0.02f);
  _voxelizer.filter(*cloud_filtered);

  //update cloud
  _cloud->clear();
  *_cloud = *cloud_filtered;
}

void Object::updateOccupancy(const Eigen::Isometry3f &T, const PointCloud::Ptr & cloud){

  if(cloud->empty())
    return;

  octomap::Pointcloud scan;
  for(const Point& pt : cloud->points)
    scan.push_back(pt.x,pt.y,pt.z);

  octomap::point3d sensor_origin(T.translation().x(),T.translation().y(),T.translation().z());
  //std::cout << T.operator()(0,0) << std::endl;
  //std::cout << T.linear() << std::endl;
  float cameraYawAngle=atan2 (T.operator()(1,0),T.operator()(0,0));  //#TODO check if "T.linear().eulerAngles(0, 1, 2)[2];" is the same
  //std::cout << " Z angle: " << cameraYawAngle << std::endl;
  //Eigen::AngleAxisf x;
  //x.fromRotationMatrix(T.linear());
  //std::cout << "AngleZ... " << x.axis() << std::endl;
  //rotationAngles.fromRotationMatrix(T.linear());
  _octree->insertPointCloud(scan,sensor_origin);
  

  
  octomap::Pointcloud wall_point_cloud; //  wall_point_cloud will represent the sensor FoV in global coordinates.
  octomap::point3d wall_point(1,0,0);    //  each point3d to be inserted into Pointwall

  for(int y=1;y<321;y++){
    for(int z=1;z<241;z++){
      wall_point.y()= (-0.560027)+(y*0.003489);
      wall_point.z()= (-0.430668)+(z*0.003574);
      wall_point_cloud.push_back(wall_point);
    }
  }

  octomath::Vector3 translation(0,0,0);
  float roll=atan2(_position.y()-sensor_origin.y(),_position.x()-sensor_origin.x());
  //std::cout << " yawn: " << roll << std::endl;
  octomath::Quaternion rotation(0,0,-cameraYawAngle);
  octomap::pose6d isometry(translation,rotation);
  wall_point_cloud.transform(isometry);

  //>>>>>>>>>> Create background wall to identify known empty volxels <<<<<<<<<<

  /*	A background wall is built leaving empty the shadow of the object, this is
      necesary so that the octree can recognize what area is empty known and
      unknown, otherwise it will assume all tree.writeBinary("check.bt");surroundings of the cloud as unknown.  */
  
  float alpha;	//	Angle in xy plane from sensorOrigin to each point in Pointwall
  float beta;		//	Elevation angle from sensorOrigin to each point in Pointwall
  float xp, yp, zp;		//	x,y,z coordinates of each point in Pointwall expressed in sensorOrigin coordinates
  float leg_adjacent_point_wall;		//	Leg adjacent length of a right triangle formed from sensorOrigin to each point in Pointwall
  float leg_adjacent_background_point;		//	Leg adjacent length of a right triangle formed from sensorOrigin to the new background point
  float distance;		//  Distance from the sensorOrigin and the new background point
  octomap::Pointcloud background_wall;     //  Pointcloud holding the background wall
  octomap::point3d iterator; //  Helper needed for castRay function

  //  distance will be computed so that the wall is always behind the object
  //  distance = 2D_Distance-Centroid-FarthermostPointInBBox + offset + 2D_Distance-sensorOrigin-Centroid
  float OFFSET=0.1;
  Eigen::Vector3f squared_distances;
  squared_distances[0]=pow(_position.x()-(_max.x()+OFFSET),2);
  squared_distances[1]=pow(_position.y()-(_max.y()+OFFSET),2);
  distance=sqrt(squared_distances[0]+squared_distances[1]);
  squared_distances[0]=pow(_position.x()-sensor_origin.x(),2);
  squared_distances[1]=pow(_position.y()-sensor_origin.y(),2);

  distance+=sqrt(squared_distances[0]+squared_distances[1]);
  
  for(int i=0;i<wall_point_cloud.size();i++){
      //std::cout << ".";

    if(!_octree->castRay(sensor_origin,wall_point_cloud.getPoint(i),iterator,false,distance)){

      //	Transform pointwall point to sensorOrigin coordinates subtracting sensorOrigin
      xp=wall_point_cloud.getPoint(i).x();
      yp=wall_point_cloud.getPoint(i).y();
      zp=wall_point_cloud.getPoint(i).z();

      //	Get alpha and beta angles
      alpha=atan2(yp,xp);
      leg_adjacent_point_wall=sqrt((xp*xp)+(yp*yp));
      beta=atan2(zp,leg_adjacent_point_wall);

      //	Get the new background points and return to global coordinates by adding sensorOrigin
      iterator.z()=sensor_origin.z()+distance*sin(beta);
      leg_adjacent_background_point=sqrt((distance*distance)-(zp*zp));
      iterator.y()=sensor_origin.y()+leg_adjacent_background_point*sin(alpha);
      iterator.x()=sensor_origin.x()+leg_adjacent_background_point*cos(alpha);

      background_wall.push_back(iterator);		//	add points to point cloud
    }
  }

 
  // std::cout << " Raytrace completed! " <<std::endl;

  _octree->insertPointCloud(background_wall,sensor_origin);    
   
  octomap::point3d p;
  Point pt;
  _occ_voxel_cloud->clear();
  _fre_voxel_cloud->clear();
  _ocupancy_volume=0.0; 
  
  OFFSET+=-0.01;



  for(octomap::OcTree::leaf_iterator it = _octree->begin_leafs(),end=_octree->end_leafs(); it!= end; ++it) {  

    p = it.getCoordinate();
    octomap::OcTreeNode * iteratorNode=_octree->search(it.getKey());
    if(!inRange(p.x(),p.y(),p.z(),OFFSET)){
      _octree->deleteNode(it.getKey());
    }else if (iteratorNode->getOccupancy()>0.49){ // occupied voxels 
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      _occ_voxel_cloud->points.push_back(pt);
      _ocupancy_volume+=pow(it.getSize(),3);
    }
    else { // free voxels
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      _fre_voxel_cloud->points.push_back(pt);
    }
    
  }

  _last_processed_view.x()=sensor_origin.x();
  _last_processed_view.y()=sensor_origin.y();

  _fre_voxel_cloud->width = _fre_voxel_cloud->size();
  _fre_voxel_cloud->height = 1;

  _occ_voxel_cloud->width = _occ_voxel_cloud->size();
  _occ_voxel_cloud->height = 1;
}
