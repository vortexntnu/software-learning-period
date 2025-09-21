cmake_minimum_required(VERSION 3.8)
project(object_detection_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)        # <-- add this
find_package(OpenCV 4.5.4 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)

add_executable(object_detection_example_node
  src/object_detection_example_ros.cpp src/utils.cpp)

add_executable(republish_color_image_publisher
  src/republish_color_image.cpp)

target_link_libraries(object_detection_example_node PUBLIC
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_include_directories(object_detection_example_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(object_detection_example_node PUBLIC
  rclcpp
  cv_bridge
  sensor_msgs
  vision_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
)

# --- add deps for the republisher target (fixes the rclcpp include error)
ament_target_dependencies(republish_color_image_publisher PUBLIC
  rclcpp
  cv_bridge
  sensor_msgs
)

# If you later include PCL/OpenCV headers in republish_color_image.cpp, add:
# target_include_directories(republish_color_image_publisher PUBLIC
#   ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# target_link_libraries(republish_color_image_publisher PUBLIC
#   ${OpenCV_LIBS} ${PCL_LIBRARIES})

install(TARGETS
  object_detection_example_node
  republish_color_image_publisher        # <-- install it too
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
