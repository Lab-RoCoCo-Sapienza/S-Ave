add_library(object_detector_library SHARED
  detection.h detection.cpp
  model.h model.cpp
  object_detector.h object_detector.cpp
)

target_link_libraries(object_detector_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

