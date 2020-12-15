add_library(logical_camera_plugin SHARED 
  logical_camera_plugin.cpp logical_camera_plugin.h
)

add_dependencies(logical_camera_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(logical_camera_plugin 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

