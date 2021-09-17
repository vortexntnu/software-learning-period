cmake_minimum_required(VERSION 3.0.2)
project(auto_talker_cpp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
    src/talker.cpp
    src/talker_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
