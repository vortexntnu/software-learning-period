cmake_minimum_required(VERSION 3.5)
project(frames)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dynamic_transform_publisher_node
  src/dynamic_transform_publisher_ros.cpp
)

target_include_directories(dynamic_transform_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamic_transform_publisher_node
  rclcpp
  tf2_ros
  geometry_msgs
)

install(
    TARGETS dynamic_transform_publisher_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
    launch
    description
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()