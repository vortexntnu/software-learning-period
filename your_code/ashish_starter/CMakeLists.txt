cmake_minimum_required(VERSION 3.5)
project(ashish_starter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transit_msgs REQUIRED)

include_directories(include)

# The node classes go in one library, and each executable is just a main()
# that starts one of them. Same split as vortex-auv's los_guidance package.
set(LIB_NAME ashish_starter_lib)
add_library(${LIB_NAME} SHARED
  src/vehicle_node_ros.cpp
  src/signal_node_ros.cpp
)

ament_target_dependencies(${LIB_NAME}
  rclcpp
  transit_msgs
)

add_executable(vehicle_node src/vehicle_node.cpp)
target_link_libraries(vehicle_node ${LIB_NAME})

add_executable(signal_node src/signal_node.cpp)
target_link_libraries(signal_node ${LIB_NAME})

install(TARGETS
  ${LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  vehicle_node
  signal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY # NB! Remember to install the launch and config directories
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
