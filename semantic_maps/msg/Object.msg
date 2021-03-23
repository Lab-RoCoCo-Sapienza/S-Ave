Header header
string label
string taxonomy
float32 probability    
float32 distance     # robot-object distance
float32 hgazebo      # height of the bounding box in Gazebo
float32 wgazebo      # width of the bounding box in Gazebo
float32 hyolo        # height of Yolo bounding box 
float32 wyolo        # width of the bounding box in Gazebo
int32 count          # number of detected objects
geometry_msgs/Vector3 min
geometry_msgs/Vector3 max
