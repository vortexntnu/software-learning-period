cmake_minimum_required(VERSION 3.8)
project(object_detection_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)

add_executable(my_image_subscriber_node
  src/my_image_subscriber.cpp
  src/utils.cpp
)

target_link_libraries(my_image_subscriber_node
  ${OpenCV_LIBS}
)

target_include_directories(my_image_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(my_image_subscriber_node
  rclcpp
  cv_bridge
  sensor_msgs
  std_msgs
  vision_msgs
  geometry_msgs
  pcl_conversions
)

install(TARGETS my_image_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(object_detection_example_node
  src/object_detection_example_ros.cpp
  src/utils.cpp
)

target_link_libraries(object_detection_example_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_include_directories(object_detection_example_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(object_detection_example_node
  rclcpp
  cv_bridge
  sensor_msgs
  vision_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
)

install(TARGETS object_detection_example_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
