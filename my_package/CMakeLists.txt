cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_publisher_node src/my_publisher.cpp)
ament_target_dependencies(my_publisher_node rclcpp std_msgs)

# Finn headers i build og install
target_include_directories(my_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS my_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# (Valgfritt, men lurt) installer headers dersom du bruker dem:
install(DIRECTORY include/ DESTINATION include)

ament_package()

# Install the executable so it can be run with `ros2 run`
install(TARGETS
  my_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
