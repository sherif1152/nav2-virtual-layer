cmake_minimum_required(VERSION 3.5)
project(nav2_virtual_layer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)


include_directories(include)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddCircle.srv"
  "srv/AddLine.srv"
  "srv/AddPolygon.srv"
  "srv/RemoveShape.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(virtual_layer SHARED
  src/virtual_layer.cpp
)

# Dependencies
ament_target_dependencies(virtual_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_srvs
  ament_index_cpp 
)

# Link service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_link_libraries(virtual_layer
  ${cpp_typesupport_target}
)

# Plugin export
pluginlib_export_plugin_description_file(
  nav2_costmap_2d
  plugin.xml
)

# Install
install(TARGETS virtual_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(virtual_layer)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_srvs
  ament_index_cpp
)

ament_package()