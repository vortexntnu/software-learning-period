cmake_minimum_required(VERSION 3.0.2) # Don't touch this :)
project(talker_cpp) # This needs to match the project name in package.xml - don't touch this either :)

# We use C++11, so make sure this is uncommented :)
add_compile_options(-std=c++11)

# Here we specify standard ROS components that we use. If you wish to
# use #include <some_ROS_standard_component>, it needs to go here.
# By "finding" the packages we include them in this project, while
# catkin_package declares things to be passed to dependent projects
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
)
catkin_package(
    INCLUDE_DIRS include    # This is the include folder we have made in the package
    CATKIN_DEPENDS          # Packages from find_package()
    roscpp
    std_msgs
    geometry_msgs
)

# To make sure the compiler finds the header files we create, include/ needs to be put here
# ${catkin_INCLUDE_DIRS} is added to make sure any other catkin dependency we have is met
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# This may come in handy if there are other external dependencies that need to be installed
# before anything else, i.e. custom messages or reconfigure servers.
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# The first argument here is the name of the executable that your C++ files will compile down to.
# The .cpp files added to this call are the files you've written and wish to compile.
add_executable(${PROJECT_NAME}_node
    src/talker.cpp
    src/talker_node.cpp
)

# Compilation dependencies are added here, but you generally don't need to touch this.
add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# Any libraries needed in the linking process is added here, you generally don't need 
# to touch this either.
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
