cmake_minimum_required(VERSION 3.8)
project(my_package2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Executable
add_executable(subscriber_node src/my_subscriber.cpp)

# Include dirs
target_include_directories(subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencies
ament_target_dependencies(subscriber_node
  rclcpp
  std_msgs
)

# Install the executable
install(TARGETS subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Ament package declaration
ament_package()
