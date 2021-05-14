#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <semantic_maps/ObjectArray.h>
#include <semantic_maps/Object.h>
#include <visualization_msgs/Marker.h>
#include <srrg_types/types.hpp>
#include <geometry_msgs/Transform.h>

typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > Isometry3fVector;
typedef std::vector<std::string> StringVector;


ros::Publisher marker_pub;
semantic_maps::ObjectArray semantic_map; // vector of Object messages
Isometry3fVector candidate_view;
std::vector<Isometry3fVector> candidate_views_list;
StringVector processed_objects_list;
std::queue<Eigen::Vector3f> views;

Eigen::Isometry3f camera_pose = Eigen::Isometry3f::Identity();
int CANDIDATE_NUM = 8;

std::string semantic_exploration;
int save = 1; 



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* 
input: robot_pose (it will contain position and orientation of the robot)
output: bool (true if tfListener is computed without any errors)
*/
bool listenRobotPose(Eigen::Isometry3f &robot_pose){
  tf::TransformListener listener;
  tf::StampedTransform robot_tf;
  try {
    listener.waitForTransform("map",
                              "base_link",
                              ros::Time(0),
                              ros::Duration(100));
    listener.lookupTransform("map",
                             "base_link",
                             ros::Time(0),
                             robot_tf);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  robot_pose.setIdentity();
  robot_pose.translation().x() = robot_tf.getOrigin().x();
  robot_pose.translation().y() = robot_tf.getOrigin().y();
  robot_pose.translation().z() = robot_tf.getOrigin().z();
  Eigen::Quaternionf q;
  tf::Quaternion tq = robot_tf.getRotation();
  q.x()= tq.x();
  q.y()= tq.y();
  q.z()= tq.z();
  q.w()= tq.w();
  robot_pose.linear() = q.toRotationMatrix();

  return true;
}


/* 
This function is waiting for the message contained in the topic "/objects_taxonomy" (it has a field "objects",
which is a list of all detected objects in the environment) and is saving a copy of the content in "semantic_map"
input: -
output: bool (true if the semantic information has been retrieved)
*/
bool receiveSemanticMap(){
  boost::shared_ptr<semantic_maps::ObjectArray const> semantic_map_msg_ptr;
  semantic_map_msg_ptr = ros::topic::waitForMessage<semantic_maps::ObjectArray> ("/objects_taxonomy", ros::Duration (10));
  if(!semantic_map_msg_ptr){
    ROS_ERROR("No semantic_map message received!");
    return false;
  }

  for(int i=0; i<semantic_map_msg_ptr->objects.size(); ++i){
    semantic_map.objects.push_back(semantic_map_msg_ptr->objects[i]);
  }
  return true;
}


/* 
This function is useful to display a marker in Rviz, representing the next position of the robot
input: next_pose
output: visualization_msgs::Marker
*/
visualization_msgs::Marker makeRVizMarker(const Eigen::Vector3f& next_pose){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = next_pose.x();
  marker.pose.position.y = next_pose.y();
  marker.pose.position.z = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(next_pose.z());

  marker.scale.x = 0.25;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  return marker;
}


/* 
This function is useful to create the move base goal message 
input: next_pose
output: move_base_msgs::MoveBaseGoal 
*/
move_base_msgs::MoveBaseGoal makeMoveBaseGoal(const Eigen::Vector3f& next_pose){

  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.frame_id = "map";   // /map
  goal_msg.target_pose.header.stamp = ros::Time::now();

  goal_msg.target_pose.pose.position.x = next_pose.x();
  goal_msg.target_pose.pose.position.y = next_pose.y();
  goal_msg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_pose.z()); // (next_pose.z())

  std::cerr << "Move Base Goal: " << goal_msg.target_pose.pose.position.x << " ";
  std::cerr << goal_msg.target_pose.pose.position.y << " ";
  std::cerr << "[]" << goal_msg.target_pose.pose.orientation << "[]" << std::endl;

  return goal_msg;
}



bool findNearestObject(semantic_maps::Object& nearest_object){
  float min_dist = std::numeric_limits<float>::max();
  bool found = false; 

  for (int i=0; i<semantic_map.objects.size(); ++i) {
    const semantic_maps::Object &obj = semantic_map.objects[i]; // access the Object at index i
    
    //check if the object has been already processed
    std::vector<std::string>::iterator itt = std::find(processed_objects_list.begin(), processed_objects_list.end(), obj.label);
    if(itt != processed_objects_list.end()){
      throw std::runtime_error("[SemanticExplorer][findNearestObject]: you're messing up things!");
      continue;
    }

    found = true;
  }
  return found;
}


Eigen::Isometry3f v2t(const Eigen::Vector3f& v){
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translation() = Eigen::Vector3f(v.x(),v.y(),0.6);
    T.linear() = Eigen::AngleAxisf(v.z(),Eigen::Vector3f::UnitZ()).matrix();
    return T;
  }


/* 
The idea of this function is to take as input, the Object.msg of the element detected by the robot,
so to easily retrieve the information needed to compute the centroid of the bounding box surrounding
the ENTIRE object (max and min points of the bounding box) and to find the coordinates of the point
with maximum x and maximum y to whom we will add the CLEARANCE and the OFFSET, in order to compute the 
ray of the circle that will include the entire object. The coordinates of the centroid and the length of 
the ray will be used to compute the position of the candidate views (in this case they will be 8, as 
specified by the value associated to the CANDIDATE_NUM variable)

input: nearest_object (it is an istance of Object.msg)
output: candidate_views 
*/
Isometry3fVector generateCandidateViews_Save(semantic_maps::Object& nearest_object){

  Eigen::Vector3f squaredDistances;
  float OFFSET = 0.1;
  float CLEARANCE = 0.6; 
  
  //--------------------------------------------------------------------------------// 

  Isometry3fVector candidate_views;
  float center_x = (nearest_object.max.x + nearest_object.min.x)/2.0f;
  float center_y = (nearest_object.max.y + nearest_object.min.y)/2.0f;

  squaredDistances[0] = pow(center_x-(nearest_object.max.x + OFFSET), 2);
  squaredDistances[1] = pow(center_y-(nearest_object.max.y + OFFSET), 2);

  // get the distance between a corner and the centroid (it will be the radius)
  auto radius = sqrt(squaredDistances[0] + squaredDistances[1]) + CLEARANCE;


  //--------------Semantic - Active vision Exploration (fix center) ----------------//

  // get the centroid of the bb of the object
  float centroid_x = center_x;
  float centroid_y = center_y;
  
  std::cout << "[INFO] CENTROID AT: " << centroid_x << " - " << centroid_y << std::endl;


  for(int i=0; i < CANDIDATE_NUM; i++){
    float alpha = i * (2*M_PI/((float)CANDIDATE_NUM));
    float x = radius * cos(alpha);
    float y = radius * sin(alpha);
    float theta = atan2(-y,-x);

    Eigen::Isometry3f T = v2t(Eigen::Vector3f(centroid_x + x, centroid_y + y, theta));

    candidate_views.push_back(T);
  }
  
  return candidate_views;
}

Isometry3fVector generateCandidateViews_ave(semantic_maps::Object& nearest_object){

  Eigen::Vector3f squaredDistances;
  float OFFSET = 0.1;
  float CLEARANCE = 0.6; 
  
  float ratio = nearest_object.hyolo/nearest_object.wyolo;

  Isometry3fVector candidate_views;
  float center_x = (nearest_object.max.x + nearest_object.min.x)/2.0f;
  float center_y = (nearest_object.min.y + ratio*nearest_object.min.y)/2.0f;

  squaredDistances[0] = pow(center_x-(nearest_object.max.x + OFFSET), 2);
  squaredDistances[1] = pow(center_y-(nearest_object.max.y + OFFSET), 2);

  // get the distance between a corner and the centroid (it will be the radius)
  auto radius = sqrt(squaredDistances[0] + squaredDistances[1]) + CLEARANCE;


  float centroid_x = center_x;
  float centroid_y = center_y;
  
  std::cout << "[INFO] CENTROID AT: " << centroid_x << " - " << centroid_y << std::endl;


  for(int i=0; i < CANDIDATE_NUM; i++){
    float alpha = i * (2*M_PI/((float)CANDIDATE_NUM));
    float x = radius * cos(alpha);
    float y = radius * sin(alpha);
    float theta = atan2(-y,-x);

    Eigen::Isometry3f T = v2t(Eigen::Vector3f(centroid_x + x, centroid_y + y, theta));

    candidate_views.push_back(T);
  }
  
  return candidate_views;
}


// -------------------------------------------------------

int main(int argc, char **argv){

  ros::init(argc,argv,"s_ave_node");
  ros::NodeHandle nh;
  nh.param<std::string>("/s_ave_node/semantic_exploration", semantic_exploration, "s-ave");	
  //semantic_exploration.compare(2,3, "s-ave") != 0
  save = (semantic_exploration.compare("s-ave") == 0);
  
  MoveBaseClient ac("move_base", true);

  marker_pub = nh.advertise<visualization_msgs::Marker>("goal_visualization_marker", 1);

  Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.translation() = Eigen::Vector3f(0.21, 0.0, 0.503); //0.0 0.0 0.5

  int current_views = 0;

  while(!listenRobotPose(robot_pose) || !receiveSemanticMap()){
    ROS_INFO("Waiting to for robot_pose and semantic_map to start exploration...");
  }
  //camera_pose = robot_pose * camera_offset;
  ROS_INFO("Starting exploration!!!");

  bool exit = false;
  while(ros::ok() && !exit) {

    //find nearest object
    semantic_maps::Object nearest_object;  // Object message
    if(!findNearestObject(nearest_object)){
      ROS_INFO("No objects to process!");
      exit = true; //no more objects to process
      continue;
    }
    std::cerr << "Processing: " << nearest_object.label.c_str() << std::endl;
    
    
    bool seen = false;
    while(!seen){
 
      //listen robot pose
      if(!listenRobotPose(robot_pose))
        continue;
      else{
        ROS_INFO("Received robot pose!");
      }

      //receive semantic map
      if(!receiveSemanticMap())
        continue;
      else{
        ROS_INFO("Received semantic map!");
      }
      
      //retreive objects from semantic mapper
      camera_pose = robot_pose * camera_offset;

      std::cerr << "Retreived! " << nearest_object.label.c_str() << std::endl;
      /*****************************/
      int object_number = 0;
      auto pos = std::find(processed_objects_list.begin(), processed_objects_list.end(), nearest_object.label);

      //  Check if we already have the candidates of this object (useful for more than one view of the same object)
      if (pos != processed_objects_list.end()){
        object_number = std::distance(processed_objects_list.begin(), pos);
        std::cerr << "Model " << nearest_object.label.c_str() << "previously processed, stored in " << object_number << std::endl;
        candidate_view = candidate_views_list[object_number];
       
      } else { 
        std::cerr << "First time running the model" << nearest_object.label.c_str() << "generating CandidateViews... " << std::endl;
        //generate candidate views
        if (save){
            candidate_view = generateCandidateViews_Save(nearest_object); //  use S_AvE
            std::cerr << "used S_AvE" << std::endl;
        } else {
            candidate_view = generateCandidateViews_ave(nearest_object); //  use AvE
            std::cerr << "used AvE" << std::endl;
        }
        candidate_views_list.push_back(candidate_view);
        processed_objects_list.push_back(nearest_object.label);
        current_views = 0;
      }

      float Candidates[CANDIDATE_NUM][4];
      for(int i=0; i<candidate_view.size(); i++){
        const Eigen::Isometry3f& T = candidate_view[i];
        
        Candidates[i][0]=T.translation().x();       // [x]
        Candidates[i][1]=T.translation().y();       // [y]
        Candidates[i][2]=T.translation().z();       // [z]
        Candidates[i][3]=T.linear().eulerAngles(0, 1, 2)[2];


        std::cout<< std::endl<<"from (" << Candidates[i][0] << ") ("<< Candidates[i][1] << ") ("<< Candidates[i][2]<< ") Angle: "<< Candidates[i][3]<<" VI: "<<Candidates[i][4];

        //current NBV
        Eigen::Vector3f NBVpos;
        NBVpos[0]=Candidates[current_views][0];
        NBVpos[1]=Candidates[current_views][1];
        NBVpos[2]=Candidates[current_views][3];     // just need 2D
        views.push(NBVpos);
      }

     /*****************************/
      //compute NBV
      ROS_INFO("evaluate NBV candidates!");

      bool reached = false;
      int index = CANDIDATE_NUM;
      while(!views.empty() && !reached && current_views < 1){

        Eigen::Vector3f nbv = views.front();

        //check if the object has been seen from all candidate views
        if(current_views == CANDIDATE_NUM){
          seen = true;
          ROS_INFO("%s: processed!",nearest_object.label.c_str());
          break;
        }


        //visualize next pose (RViz)
        visualization_msgs::Marker marker = makeRVizMarker(nbv);
        marker_pub.publish(marker);

        //send move_base goal
        ROS_INFO("Waiting for move_base action server to start.");
        ac.waitForServer();
        move_base_msgs::MoveBaseGoal goal = makeMoveBaseGoal(nbv);
        ROS_INFO("Action server started, sending goal.");
        ac.sendGoal(goal);

        // wait for the action server to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
        if (true){
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s", state.toString().c_str());          

          if(state.toString() == "SUCCEEDED"){
            reached = true;
            current_views++;
            std::cerr << "VIEWS COUNTER = " << current_views << std::endl;
          }
        }
        views.pop();
      }
      if(views.empty() || current_views == 1){
        seen = true;
        ROS_INFO("%s: processed!",nearest_object.label.c_str());
      }
    }
  }

  return 0;
}

