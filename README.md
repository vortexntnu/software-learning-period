# Software Learning Period (C++ Edition)

This is a tutorial for fundamental concepts in ROS 2. You will learn how to:

  * Set up a workspace
  * Run a node
  * Listen to a topic
  * Create a C++ node for publishing to a topic
  * Create a C++ node for subscribing to the same topic
  * Create a launch file for automatically launching the nodes

Commands needed for the tutorial can be found at the bottom of the document.

### **Installing ROS 2 and VS Code**

Follow the steps in the ROS 2 installation guide for your OS:
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

To install Visual Studio Code on your Linux computer:
[https://code.visualstudio.com/download](https://code.visualstudio.com/download)

### **For Mac and other non-Linux OS**

If you're not using a native Ubuntu setup — or want a reproducible environment — you can use Docker to run this tutorial.

We’ve provided a ready-to-use Docker environment inside the [docker/](docker/) folder of this repository. It includes:

- A Dockerfile tailored for ROS 2 development
- Easy build and run scripts (build.sh, run.sh)
- Workspace mounting between host and container
- You can use this setup on macOS, Windows (via WSL 2), or Linux.

To get started, follow the instructions in the [docker/README.md](docker/README.md).

-----

## Preliminaries

ROS 2 (Robot Operating System 2) is middleware for robotics, enabling communication between systems using messages and services. ROS 2 supports distributed systems, multiple languages (C++, Python), and modern networking.

**Colcon** is the build system for ROS 2. Nodes are executables organized in packages. Nodes communicate over **topics** (streams of messages). ROS 2 supports custom messages and services.

**Recommended:** Review ROS 2 documentation and tutorials:
[https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)
[https://rsl.ethz.ch/education-students/lectures/ros.html](https://rsl.ethz.ch/education-students/lectures/ros.html)

-----

## Task 1: Running the nodes in the repository

You need at least two terminal windows:

  * One terminal to build and run nodes
  * One terminal to listen to topics

### **Task 1.1: Creating and building a ROS 2 workspace**

A ROS 2 workspace is a folder for modifying, building, and installing packages. To create a workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

Clone the repository:

```bash
cd src
git clone https://github.com/vortexntnu/software-learning-period.git
cd ..
```

Build the workspace:

```bash
colcon build
```

Source the ros2 installation:

```bash
source /opt/ros/humble/setup.bash
```
Add the command to your shell startup script
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
For `zsh` users, replace `~/.bashrc` with `~/.zshrc`.

Source the workspace:

```bash
source install/setup.bash
```

Setting up `colcon` argcomplete is recommended to achieve inner peace 🙏:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Next, you need to "source" the autocomplete script in your shell's configuration file. This will ensure that the autocomplete functionality is loaded every time you open a new terminal.

If you are using the Bash shell (the default for Ubuntu), add a line to your `.bashrc` file. Run the following command in your terminal:

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

For `zsh` users, replace `~/.bashrc` with `~/.zshrc`.

### **Task 1.2: Running a node**

Use the `ros2 run` command with the following syntax:

```bash
ros2 run [package_name] [executable_name]
```

Example:

```bash
ros2 run talker talker_node
```

### **Task 1.3: Listening to a topic**

First, list the available topics to see what you can listen to:

```bash
ros2 topic list
```

Then, listen to a specific topic using the `echo` command:

```bash
ros2 topic echo [topic_name]
```

Example:

```bash
ros2 topic echo /talker_cpp
```

-----

## Task 2: Create a package containing a C++ publisher

You will learn to create packages and use git for version control.

### **Task 2.1: Making a branch and a ROS 2 package**

Branch out from main to keep your work organized:

```bash
git checkout -b "yourName/Task_2"
```

Create a C++ package using the `ament_cmake` build type:

```bash
ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp std_msgs
```

### **Task 2.2: Writing a C++ publisher**

Add your C++ source file (e.g., `my_publisher.cpp`) to the `my_package/src/` directory.

Next, you must update `CMakeLists.txt` to build your code. Add the following lines to `CMakeLists.txt` to define your executable and link it to its dependencies:

```cmake
add_executable(my_publisher_node src/my_publisher.cpp)

# Link the executable to its dependencies.
# You must list every package that you #include headers from.
ament_target_dependencies(my_publisher_node
  rclcpp
  std_msgs
)

# Install the executable so it can be run with `ros2 run`
install(TARGETS
  my_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
```

Learning CMake takes practice. A great way to start is by examining the CMakeLists.txt files in the `talker` and `listener` packages. Focus on understanding the essential commands that appear in most projects, as these are the fundamentals you'll use regularly.

Now, build your workspace from the root (`~/ros2_ws`):

```bash
colcon build
source install/setup.bash
```

Run your new node:

```bash
ros2 run my_package my_publisher
```

In a separate terminal, check the data being published:

```bash
ros2 topic echo /your_topic_name_here
```

### **Task 2.3: Using git**

Stage and commit your changes to save your progress:

```bash
git add <filename>
git commit -m "A descriptive commit message."
git push
```

-----

## Task 3: Create a package containing a C++ subscriber

Create a new package and a C++ node that subscribes to the topic you published to earlier. Follow the same steps as in Task 2 for creating a package and adding an executable to `CMakeLists.txt`.

### **Task 3.1: Creating a launch file**

Review the example setup in the `simple_publisher` package to see how launch files can be used to launch multiple nodes.

A launch file allows you to run multiple nodes with a single command. Create a `launch/` directory inside your package (`my_package/launch/`). Inside, create a launch file (e.g., `my_launch_file.launch.py`). The launch file can be placed in either package and launch nodes from both packages.

Run both your newly created nodes using the launch file:

```bash
ros2 launch my_package my_launch_file.launch.py
```
Also try creating a config file that is used by your launch file.

-----

### Neat Commands

Use the **Tab** key to autocomplete commands. If autocomplete fails, check that you have sourced your workspace (`source install/setup.bash`) or look for typos.

| Command                                                    | Description                                            |
| ---------------------------------------------------------- | ------------------------------------------------------ |
| `cd [folder]`                                              | Change to a different directory.                       |
| `cd ..`                                                    | Go up one directory level.                             |
| `ls`                                                       | List the contents of the current directory.            |
| `cat [filename]`                                           | Print the contents of a file to the terminal.          |
| `touch [filename]`                                         | Create a new, empty file.                              |
| `nano [filename]`                                          | Edit a file in the terminal using the nano editor. (ctrl + X) to exit.     |
| `ros2 topic echo /topic_name`                              | Listen to messages on a specific topic.                |
| `ros2 topic pub /topic std_msgs/msg/String "data: 'Hello'"` | Publish a one-time message to a topic from the CLI.    |
| `colcon clean`                                             | Remove build artifacts to prepare for a clean build.   |
| `colcon build`                                             | Build all packages in the workspace.                   |