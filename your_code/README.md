# This is a tutorial to how some fundamental concepts of ROS works. More spesifically, you will learn how to:
* set up a workspace 
* create a node for publishing to a topic
* create a node for subscribing to the same topic
* publish to the topic from the terminal
* running ROS and the nodes
* create a launch file for automatically running the nodes

## creating a workspace
A workspace is a set of directories (folders) where pieces related to ROS code are stored.
To create a workspace, open the terminal and run the following command:

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

To be able to run the code, it must be built:



## To run the nodes, a roscore must run: type roscore in the terminal
