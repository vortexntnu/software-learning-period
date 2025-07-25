cmake_minimum_required(VERSION 3.5)
project(simple_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_publisher src/simple_publisher_ros.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

target_include_directories(simple_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
    simple_publisher
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY # NB! Remember to install launch and config directories
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

# Removed redundant installation block for the config directory.

ament_package()
