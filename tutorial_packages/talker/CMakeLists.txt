cmake_minimum_required(VERSION 3.5) # Minimum CMake version for ROS2/ament_cmake
project(talker) # This must match the project name in package.xml

#This sets the C++ standard for the project. Vortex uses mostly C++20. For ubuntue 22 C++20 requires a compiler upgrade.
# Therefore we set it to C++17 for the sake of this tutorial.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# In ROS2, ament_cmake is the build system. We find all required dependencies here.
find_package(ament_cmake REQUIRED) # ament_cmake replaces catkin in ROS2
find_package(rclcpp REQUIRED)      # ROS2 C++ client library
find_package(std_msgs REQUIRED)    # Standard message types
find_package(geometry_msgs REQUIRED) # Geometry message types

# Add your executable here. The first argument is the name of the binary that will be installed.
# The .cpp files listed are the source files to compile.
add_executable(talker_node src/talker_ros.cpp)

# Make sure the compiler can find your headers located under include/ during build.
target_include_directories(talker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ament_target_dependencies is the ROS2 way to declare dependencies for your target.
ament_target_dependencies(talker_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install your executable so it can be found by ros2 run.
install(TARGETS talker_node
  DESTINATION lib/${PROJECT_NAME}
)

# This macro marks the package as an ament package (required for ROS2).
ament_package()
