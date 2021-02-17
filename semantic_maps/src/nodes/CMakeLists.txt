# platform_node
add_executable(platform_node
  platform_node.cpp
)

target_link_libraries(platform_node
  ${catkin_LIBRARIES}
)


# spatial_knowledge_node
add_executable(spatial_knowledge_node
 spatial_knowledge_node.cpp
)

target_link_libraries(spatial_knowledge_node
  ${catkin_LIBRARIES}
)

# evaluation_node
add_executable(evaluation_node
 evaluation_node.cpp
)

target_link_libraries(evaluation_node
  ${catkin_LIBRARIES}
)





