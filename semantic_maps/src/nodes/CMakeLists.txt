# simulated_platform_node
add_executable(simulated_platform_node
  simulated_platform_node.cpp
)

target_link_libraries(simulated_platform_node
  ${catkin_LIBRARIES}
)


# simulated_ideal_object_detector_node
add_executable(simulated_ideal_object_detector_node
 simulated_ideal_object_detector_node.cpp
)

target_link_libraries(simulated_ideal_object_detector_node
  semantic_mapper_library
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(simulated_ideal_object_detector_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# simulated_spatial_knowledge_node
add_executable(simulated_spatial_knowledge_node
 simulated_spatial_knowledge_node.cpp
)

target_link_libraries(simulated_spatial_knowledge_node
  ${catkin_LIBRARIES}
)



