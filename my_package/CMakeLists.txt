cmake_minimum_required(VERSION 3.8)
project(my_package)

# Default to C++17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings (optional but good practice)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create the executable
add_executable(my_publisher_node src/my_publisher.cpp)

# Make sure the compiler can find headers in include/
target_include_directories(my_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Link dependencies
ament_target_dependencies(my_publisher_node
  rclcpp
  std_msgs
)

# Install the executable so it can be run with `ros2 run my_package my_publisher_node`
install(TARGETS my_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (optional but recommended if you want to export your class)
install(DIRECTORY include/
  DESTINATION include
)

# Let ament know this package exists
ament_package()
