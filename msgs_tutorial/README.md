# messages tutorial
This tutorial will teach you:
- What messages are
- Different types of messages
- How to create and publish messsages

## What is a message?
Earlier in the learning period, you learned how to publish data from a node. The data you published, for example a string or an int, are types of messages. Sometimes, it can be advantagious to publish a compound of different data, for example if you want to publish a coordinate in the xy-plane. In this case a message consisting of two floats makes much more sense than two separate messages, each consisting of one float. A message can therefore be a single data type or a compound of several data types (much alike structs/classes). Note that a message do not have to contain only one data type, sometimes you want a message that can represent a string and an int.


## ROS inbuilt messages
Ros has a lot of predefined messages from several libraries. Some of the libraries are std_msgs, geometry_msgs, actionlib_msgs. 
To get an overview of all message types, use the command 
```
rosmsg list
```

To get information about a specific type of message, use the command 
```
rosmsg info library/type
```

For example
```
rosmsg info geometry_msgs/Point
```

### Standard messages (std_msgs)
Std_msgs are the most primitive messages, and consist of only one variable. Some examples are String, Int32, Int64, Float32, Bool, Char. Note that when defining variables of these data types, the name of the data types must be std_msgs/Int32, and so on. Also note the capital letters.

### Composite messages
Geometry_msgs (amongst others) are composite messages. This means that they consist of more than one variable. The geometry_msgs/Point, consisting of the variables float64 x, float64 y and float64 z, is one example.
An even more complex message type is the geometry_msgs/Pose. This message has attributes of types geometry_msgs/Point and geometry_msgs/Quaternion, where Point gives information about position and Quaternion gives information about orientation. geometry_msgs/Pose is a nested msg, because both Point and Quaternion are msgs in themselves, with their own attributes.


## Custom messages
Sometimes you want to have a specific msg type that does not exist in ROS. You can then create this msg, a  custom msg, yourself. In Vortex, we try to keep the number of custom msgs limited, but sometimes they are necessary, and it is also nice to know that they exist. When running a node that publishes a custom msg, you first have to wource your workspace. The reason why you have to do this with custom msgs but not with predefined ROS msgs, is that the custom msgs are located in your workspace.


## Tasks
### Task 1 - publishing predefined messages
1. Create a package and a node for publishing.
2. Publish a message of a chosen type.
    - First, you have to create an empty msg of the chosen type, then set values for the attributes in the empty msg and at last publish the msg.
    - Tip 1: use the commands mentioned earlier to list all msg types and to get information about a msg type. You can also use Google to find this information.
    - Tip 2: to be able to use msgs, they must be included. Example from a .py file:
    ```
    from geometry_msgs.msg import Point
    ```
3. Optional, but recommended to get to know the different types of msgs: repeat step 2, where you create several publishers that publish different types of msgs.

### Task 2 - creating and publishing custom messages
1. Create a package called my_msgs. In this package, create a folder called msg.
2. Create a file of type .msg inside the msg folder and give the file a suitable name for what you want the msg to represent.
3. Create the msg by defining attributes (just like you would define normal variables): 
```
datatype variable_name
```
For inspiration, you can look in vortex-asv/vortex_msgs/msg on github.
4. Create a new publisher in the node from task 1.
    - Tip 1: if you would like to use the msg you must add the package containing the msg as a dependency to the package where your node is located. This is because the msg is located in another package.
    - Tip 2: you can find out how to manage the CMakelists.txt and package.xml in ros tutorials, for example: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
5. Build and run your code.
6. Optional: create different custom msgs and publish them.

