# Services tutorial

 This tutorial will teach you:

* What a ros service is
* How to create custom services
* How to use services


## What is a ros service?

In a subscriber or publisher you are working with a continoues flow of messages. But what if you only want to send a single message? This is where ros services come in.

You can think of services as a server client model. You have node where you define your service, this will act as your server. The service will only be executed if the node recieves a request from a client. This client is a node that is sending a request to the server. You can also send a request through terminal.

Services differ from nodes in that you use two messages. One request message, and one return message.

These messages act as structs, and are defined in a .srv file. The .srv file is composed in two parts, the request and the return value you will recive from the server.

Example of a service file used in vortex-asv:

Waypoint.srv

```
#request
float32[] waypoint
---
#response
bool success
```

This is a custom service that sends a list of floats as a request to the server, and the server will return a bool once the list is recived.

## Task 1: Make your own service

The goal is to make a sevice called AddTwoInts that is located in its own package, and create a server that is dependent on ADDTwoInts. You will then send a request to the server using a terminal.

1. Create a package for your services.

2. Create a folder inside your package called srv.

3. Create a file called AddTwoInts.srv.

4. Write the request message type such that you send two ints to the server.

5. Write a response message such that the server returns an int.

6. Create a package containing your server, that is dependent on rospy and the package containing your services.

7. Run your server and use the terminal to send a request.

Feel free to write in either python or c++.

A nice walktrough in how to do this task with Actions rather than services can be found here: https://roboticsbackend.com/ros-create-custom-action/.

You can follow this tutorial if you remove  "actionlib_msgs" in both the CMakelists and package.xml. 


Pro tip : 
Both custom and standard services are located in packages. If you would like to use a service you must add the package containing the services as a dependency to your own package.


## Neat commands

To get a list of all running services:
```
rosservice list
```
To send request to server from terminal:
```
rosservice call \service_name [request]
```

