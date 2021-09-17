cmake_minimum_required(VERSION 3.0.2)
project(embedded_talker_py)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    nav_msgs
)

# catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    nav_msgs
)

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
