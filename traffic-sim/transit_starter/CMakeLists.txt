cmake_minimum_required(VERSION 3.5)
project(transit_starter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transit_msgs REQUIRED)

add_executable(vehicle_node src/vehicle_node_ros.cpp)
target_include_directories(vehicle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(vehicle_node rclcpp transit_msgs)

add_executable(signal_node src/signal_node_ros.cpp)
target_include_directories(signal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(signal_node rclcpp transit_msgs)

install(TARGETS
  vehicle_node
  signal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY # NB! Remember to install the launch directory
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
