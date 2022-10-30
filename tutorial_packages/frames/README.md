# Frames and coordinate transformation tutorial package

This package aims to give a very basic introduction to coordinate frames and transforms in ROS1, using the tf2 package.



# Task 1: Static transforms

Goal: Create the tf tree in RViz that shows base_link together with the IMU and DVL frame.

Everything should be in NED

Solution: Define the transforms in the transforms.launch

1. Try running the example code

    roslaunch frames frames.launch

2. Use RViz to inspect your frames

TODO: Add images


2. Define the required static transforms

    <node pkg="tf2_ros" type="static_transform_publisher" name="name_of_transform" args="x y z yaw pitch roll frame_from frame_to" />


Once completed, task 1 will allow you to run the given data together with the robot_localization EKF in order to produce filtered output state estimates from the DVL and IMU



# Task 2: The endless battle between ENU and NED

Goal: Make base_link an ENU frame, and the rest NED (Robot_localization assumes ENU convention for base link. However, the sensor frames must adhere to what is physically the case for the AUV)

Solution: Swap x and y in the transforms, and apply correct rotations



# Task 3: Dynamic transforms

Goal: Use the provided data to transform something dynamically?