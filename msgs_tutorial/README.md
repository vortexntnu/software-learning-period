# Messages in ROS
Earlier in the learning period, you learned how to publish data from a node. The data you published, for example a string or an int, are types of messages. Sometimes, it can be advantagious to publish a compound of different data, for example if you want to publish a coordinate in the xy-plane. In this case a message consisting of two floats makes much more sense than two separate messages, each consisting of one float. A message can therefore be a single data type or a compound of several data types (much alike structs/classes). Note that a message do not have to contain only one data type, sometimes you want a message that can represent a string and an int.


# ROS inbuilt messages
Ros has a lot of predefined messages from several libraries. Some of the libraries are std_msgs, geometry_msgs, actionlib_msgs. 
To get an overview of all message types, use the command 
'''rosmsg list'''

To get information about a specific type of message, use the command 
'''rosmsg info library/type '''
For example
'''rosmsg info geometry_msgs/Point'''

## Standard messages (std_msgs)
Std_msgs are the most primitive messages, and consist of only one variable. Some examples are String, Int32, Int64, Float32, Bool, Char. Note that when defining variables of these data types, the name of the data types must be std_msgs/Int32, and so on. Also note the capital letters.

## Composite messages
Geometry_msgs (amongst others) are composite messages. This means that they consist of more than one variable. The geometry_msgs/Point, consisting of the variables float64 x, float64 y and float64 z, is one example.
An even more complex message type is the geometry_msgs/Pose. This message has attributes of types geometry_msgs/Point and geometry_msgs/Quaternion, where Point gives information about position and Quaternion gives information about orientation.


## Custom messages


# Tasks
## Task 1 - publishing predefined messages
Create a package and one node; the node will have a publisher and a subscriber (use the subscriber to confirm that the publisher published - this can also be done by echoing the topic).
Remember to list dependencies when creating the pgk.
Publish a message of a chosen type (tip: use the commands mentioned earlier to list all msgs and to get information about a type of msg - this can also be googled).
Tru to publish different types of msgs, and from different libraries.

# Task 2 - publishing custom messages
Create a custom msg
Create a new node in the same pgk as earlier, and publish the custom msg.
If you want, make some more custom msgs and publish them.