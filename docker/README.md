# Docker tutorial
This folder contains an example Dockerfile and docker-compose.yml that serves as an example for the basics of Docker and Docker Compose. The sections below provide an annotated view of the docker files, as well as a short guide on how to use them.

Note: The Dockerfile and docker-compose.yml are not finished, and you will need to use a fraction of what was presented in order to complete them.


## Write your Dockerfile
The `Dockerfile` is the recipe in which you will need to specify how to set up the virtual environment for our example program. 

### FROM
The first line in a Dockerfile is usually a *FROM* statement, which lets us build our image  on top of an existing one, instead of setting everything up from scratch. For our case we may simply use the base `ubuntu:20.04` image.


> Task: Set the base image to ubuntu:20.04


### RUN
Our example program requires python3 and so we will need to install it - since we are using the ubuntu base image, we may use apt to install python3.

> Task: apt install python3

<details>
  <summary>Hints</summary>

* You will likely need to apt update and apt install in the same RUN  

* The Dockerfile build step should be automated, but apt install python3 will ask you to enter Y/n. How can you automate this?  
</details>


### COPY
To access files on the host machine inside a container without setting a volume, you will need to copy the files over manually.

> Task: Copy the entrypoint.sh and example_program.py files to the root of the container

### ENTRYPOINT
The *ENTRYPOINT* statement determines the command that is executed when the image is deployed. The usual way to handle this is to let the ENTRYPOINT command run a shell script that contains the commands to run.

> Task: Set the entrypoint to run the entrypoint.sh file

## Build your local image
Building a local image is fairly easy, and although it may be somewhat time-consuming, it is a set-and-forget process. To build the example image presented here, simply navigate to this (docker/) folder and run

```
docker build . --tag "vortex:tutorial"
```
where `tag` will be the "human representation" of the hex tag, here set to *vortex:tutorial* as an example

## Deploy your local image
The simplest way to deploy an image is using the `docker run` command. It is good practice to use the `--rm` flag to ensure that docker cleans up after itself after running.

> Task: Run the vortex:tutorial image locally

You may optionally add the `-it` flags to open an interactive terminal with stdio. There are many other flags to add depending on your needs: Refer to https://docs.docker.com/engine/reference/run/ for more info.
 
## Write your docker-compose.yml file
While this example is very basic and realistically does not need any of the features that Docker Compose provides, it is still a good execise to just get familiar with the commands you are likely to use.

### Write a single service
The boilerplate required to write a service is already in place in the docker-compose.yml file.

> Task: Choose a container name and set the image to be the one we built in the previous steps

### Multiple services
You may define multiple services under the `services:` tag. Note that service names need to be unique 

> Task: Duplicate the service you have already written and deploy them simultaneously. 


### Overwriting entrypoints
If you wish to debug or otherwise inspect the container but not run the entrypoint, you may overwrite it using the `entrypoint:` field.

> Task: Replace the entrypoint of the second service to do nothing (i.e. run /bin/bash)