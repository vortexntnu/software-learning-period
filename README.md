# Software learning period

 This is a tutorial to how some fundamental concepts of ROS works. More spesifically, you will learn how to:

* set up a workspace 
* run a node
* listen to a topic
* create a node for publishing to a topic
* create a node for subscribing to the same topic
* publish to the topic from the terminal
* create a launch file for automatically launching the nodes

Commands needed for the tutorial can be found at the bottom of the document. 
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


### Task 2: Create a package containing a publisher (C++ or Python)
By the end of task 2 you will be able to make catkin packages, be familiar with what a branch is and the commands: git add, git commit and git push.

For this task you will need visual studio code on you linux computer. Download link can be found here: https://code.visualstudio.com/download

You can either look around in the training repo or head over to ROS-tutorials: http://wiki.ros.org/ROS/Tutorials if you need inspiration on how to write a publisher in either C++ or Python. In order to validate that your package works you can try to listen to the node you made. A nice tip is to use the code for a publisher in the ros wiki, and use that to see if you can get the node to run. Task 2 will mainly walk you through the "janitor work" that needs to be done in order for your node to be able to  run in your newly made catkin package.


### Task 2 prelimenaries for git

At Vortex NTNU we organize our code in repositories, and use a tool called git for version control. Each repository contains the codebase for a specific system. These repositories are stored at Github, and each of these repositories has different versions.

These versions are what we call branches, so a branch is in fact just a version of the codebase. Each repository has a version where the code runs smoothly and is able to build. This is called the master branch. 

When we want to add a different feature to our codebase, we clone the remote codebase to our computer. Once the codebase is stored locally on our computer we want to let git know that we are working on a new version of the codebase (new branch). When you make a new branch you always branch out from either the working version of the codebase (master branch), or an experimental version of the codebase (other branch). 

Once we have implemented our new feature in our new branch we want to merge our branch with the master branch. To do this you have to make a pull request. This is when the changes added by your branch will be read through by one or more software leads, and either approved or declined for mergening with the master branch. In this tutorial we want you to push your branch to the repository, but we don't want you to open a pull request.

The benefit of using git is that it is possible to revert any changes made to the codebase, and it allows for easier cooperation when working on the same codebase. 

A nice git tutorial can be found at: https://www.youtube.com/playlist?list=PL4cUxeGkcC9goXbgTDQ0n_4TBzOO0ocPR.


### Task 2.1 Making a branch and a catkin package

Branch out of the master branch by using the command:

```
git checkout -b "yourName/Task_2"
```

Once you are on your branch you can navigate to the your_code directory. Then use the "catkin_create_pkg" command:

```
catkin_create_pkg [package name] [dependency 1] [dependency 2]
```

Example:

```
catkin_create_pkg myPackage std_msgs rospy roscpp
```
The command above will make a package called myPackage which is dependent on std_msgs rospy and roscpp. "std_msgs" is a package containing all the standard messages for ros. You need the rospy package if you want to write a node in Python, and roscpp if you want to make a node in c++.  It will also generate a new folder called myPackage. This folder will contain an empty src folder, a CMakeLists.txt and a package.xml file. 

Info on the contents of the package.xml file can be found here: http://wiki.ros.org/catkin/package.xml.

Info on the content of a CMakeLists.txt can be found here: http://wiki.ros.org/catkin/CMakeLists.txt.


### Task 2.2 Writing a publisher (Python)

Once your package has been created you should rename your src folder to scripts and create a new python file inside your new scrpits folder. It is important to add the shebang "#!/usr/bin/env python" at the top of you python file. This is used to tell the kernel which interpreter should be used to run the commands present in the file. Once the shebang is added you can write your script, but you will need to make it an executable before being able to use either python3 or rosrun to execute your python file. This can be done by using the "chmod -x" command.

```
chmod -x [filname]
```

Example:

```
chmod -x myPythonNode.py
```


### Task 2.2 Writing a publisher (C++)



If you created your package and listed roscpp as a dependency, then an include folder was also created. Inside this folder there is another folder with the same name as your package. Here you can create you header file. In order to link your header file to your .cpp file (which should be located in the src folder) and make your .cpp file an executable you need to make some changes to CMakeLists.txt.

There are several lines you need to uncomment in CMakeLists.txt:

1. You want to head over to approximatly line 105 and uncomment "INCLUDE_DIRS include"

2. In include_directories you want to uncomment "include".

3. You wnat to declare your .cpp file as an executable, so head to  approximatly line 135 and uncomment "add_executable(${PROJECT_NAME}_node src/publisher_cpp.cpp". After "src/" it is important that you write the filename of the .cpp file you want to make an executable.

4. You want to uncomment "add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})"

5. Lastly you have to head to approximatly line 148 and uncomment " target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )".

Now you need to head over to your workspace folder and use catkin build, and source your workspace.

Now you can try to run the node with rosrun!

 If you would like more explanations as for how CMakeLists.txt works, you can snoop around in the CMakeLists.txt in the talker_cpp package located in tutorial packages.

###  Task 2.2 Using git
When you have created your files you can first "stage" them by using the command "git add", this makes it possible to commit the chenges later.

```
git add /filename
```

Once all your files have been staged you can commit them by using the command "git commit"

```
git commit -m"A commit message."
```

Making a commit is similar to making a savepoint, as you can revert commits. A commit always comes aling with a commit message. This message should be as precise as possible in regards to exactly what changes you have done.

Every time changes are made to a file it needs to be staged again in order for it to be included in a commit. You can stage and commit as many times as you like. You will only publish your local branch to the remote repository once you use the "git push" command.

```
git push
```

### Task 3: Create a package containing a subscriber (C++ or Python)

Make a package containing a subscriber that will subscribe to the topic that is being published from your earlier package. If you already wrote your publisher in Python you could write your subscriber in C++.

### Task 3.1

Make a launch file to launch both your nodes at once. 

If you need inspiration for how to make a launch file, it is reccomended that you inspect the launch files in the repo.

Once you have made your launchfile (which is typically located in a package) you can launch is by using the "roslaunch command".

```
roslaunch [package_name] [launchfile_name]
```

Example:

```
roslaunch tutorial_setup system_launchfile.launch
```
This command will launch the premande nodes in the training repo.
### Neat commands

You are often able to autocomplete commands by using tab while writing them. If the command does not autocomplete it usally means that something is not sourced properly, or that you have a typo in your command.

You can change directories by using the "cd" command.

```
cd [example_folder]
```

Example:

```
cd Downloads
```

You can also use cd to go out of a directory.

Example:

```
cd ..
```


You can print what subdirectories is inside your current directory by using the "ls" command.

```
ls
```


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


If you want to make something executable you can use the "chmod -x" command.

```
chmod -x myfile.py
```

If you want to listen to a topic in terminal you can use the "rostopic echo" command.

```
rostopic echo /topic
```

If you want to publish to a topic in terminal you can use the "rostopic pub" command.

Example: 
```
 rostopic pub /topic_name std_msgs/String hello
```

If you wnat to change directory to a package you can use roscd.

```
roscd [package name]
```
