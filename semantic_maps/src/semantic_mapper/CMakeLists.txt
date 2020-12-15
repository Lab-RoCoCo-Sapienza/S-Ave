add_library(semantic_mapper_library SHARED
  object.h object.cpp
  semantic_mapper.h semantic_mapper.cpp
)

target_link_libraries(semantic_mapper_library
  utils_library
  object_detector_library
  yaml-cpp
  ${OpenCV_LIBS}
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

