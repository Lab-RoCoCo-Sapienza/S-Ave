add_library(utils_library SHARED
  utils.h utils.cpp
)
target_link_libraries(utils_library
  ${catkin_LIBRARIES}
)
