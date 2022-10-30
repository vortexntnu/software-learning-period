# Frames and coordinate transformation tutorial package




# Task 1: Static transforms

Goal: Create the tf tree in RViz that shows base_link together with the IMU and DVL frame
Optional: Also add camera center frame?
Solution: Define the transforms in the transforms.launch

1. Try running the example code
    You should see an error complaining about static transforms not defined

2. Define the required static transforms

    <node pkg="tf2_ros" type="static_transform_publisher" name="name_of_transform" args="x y z yaw pitch roll frame_from frame_to" />


Once completed, task 1 will allow you to run the given data together with the robot_localization EKF in order to produce filtered output state estimates from the DVL and IMU




# Task 2