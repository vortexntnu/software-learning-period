# software-learning-period
A repository dedicated to the team 2022-2023 to mess around with Git and to learn ROS

This repo is structured similarly to how our main Vortex-AUV repo is structured. tutorial_setup is the entrypoint for launching the entire system, while the rest of the folders here contain packages related to the folder name.


## This is a tutorial to how some fundamental concepts of ROS works. More spesifically, you will learn how to:
* set up a workspace 
* run a node
* create a node for publishing to a topic
* create a node for subscribing to the same topic
* publish to the topic from the terminal
* create a launch file for automatically launching the nodes
## Installing ROS and catkin

Follow the steps in the ROS-wiki, and install the "Desktop version".

http://wiki.ros.org/noetic/Installation/Ubuntu


## creating and building a workspace
A workspace is a set of directories (folders) where pieces related to ROS code are stored.
To create a workspace, open the terminal (ctrl + Alt + T) and run the following command:

```
mkdir name_of_your_choice_ws
```
mkdir = make directory

Note that the name can be whatever you desire, however, adding "_ws" at the end makes it easy to remember that this folder is a workspace.

A certain architecture is required for the code to build and run properly. Therefore, a src folder must be created inside the workspace:

```
mkdir src
```

Move to the src folder (cd = change directory), then clone the software-learning-period repository from github:
```
cd name_of_your_choice_ws/src

git clone https://github.com/vortexntnu/software-learning-period.git

```

To be able to run the code, move to the workspace folder and build:
```
cd ../..
catkin build
```
One dot means "this folder" and two dots means "previous folder". The command in the first line therefore means that you are moving 2 layers of folders back according to the path to the folder you currently are in.

## Task 1: running the nodes in the repository

You need at least three termianl windows open for this part. 
* One terminal window to run the node (this will be the termianl that you have already built the workspace in.)
* One terminal to run a roscore
* One terminal to listen to the node

# Task 1.1 Sourcing ros and running roscore

Pick an empty terminal and begin by sourcing ros. This will allow us to use ros commands in terminal.

Sourcing ros:

```
source /opt/ros/noetic/setup.bash
```

Then run a roscore:


```
roscore
```

The roscore manages communications between nodes, and every node registers at startup with the master. You will not be able to run nodes unless a roscore is running.

# Task 1.2 Sourcing the workspace and ROS
Once the repository has been built, we need to source both ROS and the workspace.

Sourcing ros:

```
source /opt/ros/noetic/setup.bash
```

To source the workspace you need to be located in your workspace folder.

```
source devel/setup.bash
```

# Task 1.3 Running a node



It is now possible to use the command "rosrun" to run nodes. The arguments for rosrun are the package name and the nodename.

```
rosrun [package_name] [nodename]
```

Example:

```
rosrun talker_cpp talker_cpp_node 
```

The command above will run the talker_cpp_node which is located in the catkin package named talker_cpp.


# Task 1.4 Listening to the node 

Source ros in your last empty terminal:

```
source /opt/ros/noetic/setup.bash
```

Then run the command: 

```
rostopic list
```

This command will output all the different topics you can listen to.

```
$ rostopic list
/pose_cpp
/rosout
/rosout_agg
/seq_cpp
```
Underneath the "rostopic list" command, you can see the different topics being published, each beginning with "/".

Next we want to listen to the topic being published from the node by using the rostopic echo command.

```
rostopic echo [topic name]
```

Example:
```
rostopic echo /pose_cpp
```

## Task 2: Create a package containing a subscriber (C++ or Python)

## Running a node

## Creating a launch file


## To run the nodes, a roscore must run: type roscore in the terminal

