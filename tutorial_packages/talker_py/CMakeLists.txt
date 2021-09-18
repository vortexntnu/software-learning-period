cmake_minimum_required(VERSION 3.0.2) # Don't touch this :)
project(talker_py) # This needs to match the project name in package.xml - don't touch this either :)

# Here we specify standard ROS components that we use. If you wish to
# use 'import <some_ROS_standard_component', it needs to go here.
# By "finding" the packages we include them in this project, while
# catkin_package declares things to be passed to dependent projects
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    nav_msgs
)
catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    nav_msgs
)

# Uncomment this if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# Mark executable scripts for installation. This can be used as an
# alternative to the command above in order to specify specific scripts, 
# as well as selecting the installation destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
