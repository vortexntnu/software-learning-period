## The listener package

This folder is what's considered a "colcon package". A package is loosely speaking any folder that contains the package.xml and CMakeLists.txt files,
with valid "colcon package" contents. The files here can be used as a basis for new packages whenever you need to make one, especially if you plan to
write your package in C++. (In reality there are no restrictions on language for a package, but we tend to stick to writing one specific package in
one specific language)

To simplify the package creation process, you may run 

```
ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp std_msgs
```

which will create a folder with name ```my_package``` with a template package.xml and CMakeLists.txt. Note that these contain a lot of unneccesary components
(most of them are commented out, but it's a good idea to remove everything that's not used, so the files here may still come in handy). For this package, the CMakeLists.txt and package.xml also contain a summary of the documentation that comes with the templates, and should be a bit easier to follow.

Since this is a package with C++ as the primary language, it contains the src/ and include/ folders, which need to be linked to in CMakeLists.txt!
Following standard C++ practices, header files go in the include/ folder, while source code goes in src/. Note that the headers are not placed directly in include/, but rather in include/listener/. This is standard practice for all colcon packages using C++.


# Building the package
To build the package, use:
```
colcon build --packages-select listener
```
Source your workspace:
```
source install/setup.bash
```

# Running the node
To run the node:
```
ros2 run listener listener_node
```

# How it works
The code uses rclcpp and subscribes to messages to the topic `/"talker_pose"`.
- The node is implemented by inheriting from `rclcpp::Node`.
- Subscribers are created using `create_subscription<>()`.
- The `create_subscription` call requires a callback function—a function that ROS 2 will execute automatically whenever a new message arrives on the topic.
- The syntax `std::bind(&Listener::pose_cb, this, std::placeholders::_1)` connects our class's `pose_cb` method to the subscriber. It ensures that whenever a message is received, our `pose_cb` function is called on the current node instance (this), and the message itself is passed in as the argument (_1).
