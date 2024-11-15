cmake_minimum_required(VERSION 3.8)
project(clf_cbf)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(quadrotor_msgs REQUIRED)

# List of dependencies
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2_ros
  Eigen3
  quadrotor_msgs
)

set(SHARED_LIBRARIES
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcbf_dot_Ofast.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcbf_Ofast.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libclf_dot_Ofast.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libclf_Ofast.so
)

# Create a library for the node
add_library(clf_cbf_node SHARED
  src/clf_cbf_node.cpp
)

# Specify the include directories
target_include_directories(clf_cbf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
ament_target_dependencies(clf_cbf_node ${dependencies})
target_link_libraries(clf_cbf_node ${SHARED_LIBRARIES})

# Register the node as a component
rclcpp_components_register_node(clf_cbf_node
  PLUGIN "clf_cbf_node::ClfCbfNode"
  EXECUTABLE clf_cbf_node_exec)

# Install the library and executable
install(TARGETS clf_cbf_node clf_cbf_node_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY lib/
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# Export the package
ament_package()