# platform_node
add_executable(platform_node
  platform_node.cpp
)

target_link_libraries(platform_node
  ${catkin_LIBRARIES}
)


# ideal_object_detector_node
#add_executable(ideal_object_detector_node
# ideal_object_detector_node.cpp
#)

#target_link_libraries(ideal_object_detector_node
#  semantic_mapper_library
#  ${OCTOMAP_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

#add_dependencies(ideal_object_detector_node 
#  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#  ${catkin_EXPORTED_TARGETS}
#)

# spatial_knowledge_node
add_executable(spatial_knowledge_node
 spatial_knowledge_node.cpp
)

target_link_libraries(spatial_knowledge_node
  ${catkin_LIBRARIES}
)


