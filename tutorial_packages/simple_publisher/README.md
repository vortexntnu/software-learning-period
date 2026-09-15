## Simple Publisher Package

This package provides a basic example of using launch files in ROS 2. It demonstrates how to:

  - Launch nodes using launch files.
  - Read parameter values from a configuration file during launch.
  - Bonus! Learn about quality of service settings for publishers.

### Launch Files

A launch file allows us to run (launch) multiple nodes using just one terminal command. Another benefit of using launch files is that you can read parameters from a config file and use these values for the parameter of the node. The benefit of a launch file is that you dont have to recompile your package when changing the parameter values in a config file.

To enable this you need to add an argument to the standard colcon build command.

```bash
colcon build --symlink-install
```

This creates a symbolic link between you config file here in you config directory and the config file that is installed under the install directory of your workspace when building the package.

Since the launch file also launched the talker node from our other package it is a good practice to build this package with `--packages-up-to`. This will build all packages this package depends on first, ensuring all dependencies are met. `--packages-up-to` works because we have added that dependency in the `package.xml` of this package:

```xml
<depend>talker</depend>
```

The final build command is:

```bash
colcon build --packages-up-to simple_publisher --symlink-install
```

### Launch and Config Directories

The `launch` directory contains launch files that define how nodes are started and configured. The `config` directory holds parameter files used to set node parameters at launch time.

**Note:**
To ensure these directories are installed with your package, you must include them in your CMake configuration (typically in `CMakeLists.txt`) using the appropriate install commands.

```cmake
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)
```

With the `DESTINATION` set to the share folder, you can load the config file in your launch file using the following command.

```python
config_path = os.path.join(
        get_package_share_directory('simple_publisher'),
        'config',
        'simple_publisher_config.yaml'
    )
```

To run multiple nodes with the launch file we use `ros2 launch` instead of `ros2 run`

```bash
ros2 launch package_name launch_file_name.py
```

Try launching the launch file in this package and use the commands `ros2 node list`, `ros2 topic list`, and `ros2 topic echo /topic_name` to verify that both nodes have launched. Also play around with changing the values of the params in the config file and launch the nodes again to verify that the changes have taken effect.

### Launch arguments

Launch files also support a variety of logic and conditionals when launching. This example includes the use of a launch argument. Launch arguments allow us to control the behaviour of a launch file from the CLI (command-line interface).

Running the command below shows the launch arguments of this launch file.

```bash
ros2 launch simple_publisher simple_publisher_launch.py --show-args
```

To set a launch argument you simply run:

```bash
ros2 launch simple_publisher simple_publisher_launch.py name:=value
```

This example uses the argument to set the names of one of the nodes launched. Try it out and see if you can verify that it works.

### Use this package as a starting point for learning how to manage ROS 2 node parameters and launches.

### Quality of Service (QoS) Settings

Review the constructor in `src/simple_publisher_ros.cpp` to see how Quality of Service (QoS) settings are configured for publishers and subscribers in ROS 2. QoS profiles allow you to configure the settings for how messages are sent over the network in ROS 2 between publishers and subscribers.