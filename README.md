# Software learning period

 This is a tutorial to how some fundamental concepts of ROS works. More spesifically, you will learn how to:

* set up a workspace 
* run a node
* listen to a topic
* create a node for publishing to a topic
* create a node for subscribing to the same topic
* publish to the topic from the terminal
* create a launch file for automatically launching the nodes


### Installing ROS and catkin
Follow the steps in the ROS-wiki, and install the "Desktop version".

http://wiki.ros.org/noetic/Installation/Ubuntu


### Prelimenaries

ROS is short for Robot operating system. This is the middelware we use on our AUV to communicate between all our systems. ROS allows for programs to communicate over a defined API with ROS messages and services. It enables programs to run on multiple computers and communicates over the network. ROS modules can also run on different laguages such as C++ and Python.

Catkin is the build system for ROS. The tutorial will not go in depth on exactly how catkin works, but for those interested you can read up on catkin here: https://nu-msr.github.io/me495_site/lecture02_catkin.html

This tutorial will mainly focus on nodes. A node is a single purpose executable program organized in packages. Nodes communicate over topics, which again is a name for a stream of messages. 

ROS has its own standard messsages and services but it is also possible to create your own custom messages and custom services.

It is recommendedd to have a look at the ROS lecture slides at found at the university of zurich if you want a good primer to what ROS is and how it works.
https://rsl.ethz.ch/education-students/lectures/ros.html


### Task 1: Running the nodes in the repository

Task 1 will go through how to make a catkin workspace, how to run a node, and how to listen to a topic.

You need at least three terminal windows open for this part. 
* One terminal window to run the node (this will be the termianl that you will build the workspace in.)
* One terminal to run a roscore
* One terminal to listen to the node

### Task 1.1: Creating and building a catkin workspace

A catkin workspace is a folder where you modify, build, and install catkin packages. To create a workspace, open the terminal (ctrl + Alt + T) and run the following command:

```
mkdir name_of_your_choice_ws
```
mkdir = make directory

Note that the name can be whatever you desire, however, adding "_ws" at the end makes it easy to remember that this folder is a workspace.

A certain architecture is required for the code to build and run properly. Therefore a src folder must be created inside the workspace:

```
mkdir src
```

Move to the src folder (cd = change directory), then clone the software-learning-period repository from github:
```
cd name_of_your_choice_ws/src

git clone https://github.com/vortexntnu/software-learning-period.git

```

In order to compile and link the catkin packages inside the repo we need to use the command "catkin build" while located at the workspace folder you created. 
```
cd ../..
catkin build
```
One dot means "this folder" and two dots means "previous folder". The command in the first line therefore means that you are moving 2 layers of folders back according to the path to the folder you currently are in.

To read more on catkin workspaces http://wiki.ros.org/catkin/workspaces

### Task 1.2 Sourcing ros and running roscore

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

### Task 1.3 Sourcing the workspace and ROS
Once the repository has been built, we need to source both ROS and the workspace.

Sourcing ros:

```
source /opt/ros/noetic/setup.bash
```

To source the workspace you need to be located in your workspace folder.

```
source devel/setup.bash
```

### Task 1.4 Running a node

It is now possible to use the command "rosrun" to run nodes. The arguments for rosrun are the package name and the nodename.

```
rosrun [package_name] [nodename]
```

Example:

```
rosrun talker_cpp talker_cpp_node 
```

The command above will run the talker_cpp_node which is located in the catkin package named talker_cpp.


### Task 1.5 Listening to the node 

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

Your output should look like this: 

```
---
position: 
  x: 5.0
  y: 1.0
  z: 0.0
orientation: 
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
---
```

### Intermission and neat commands

You can use the "cat" command in order to print a files content to terminal. The command below will print the contents of examplefile.py to the terminal.

```
cat examplefile.py
```




You can create a new file by using the "touch" command. The command below will create a new python file in your current directory called examplefile.py.

```
touch examplefile.py
```


You can use the "nano" command to do text editiong from terminal. If nano is not installed type:  

```
sudo apt-get install nano
```

In order to do text editiing in a pythonfile:

```
nano examplefile.py
```

If you want to exit nano then you can use (ctrl + x) 
You will the be prompted if you wold like to save your changes. You can decide weather to save or dicard by typing (N) or (Y).


## Task 2: Create a package containing a subscriber (C++ or Python)

## Running a node

## Creating a launch file


## To run the nodes, a roscore must run: type roscore in the terminal

