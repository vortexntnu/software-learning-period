# Docker tutorial
This folder contains an example Dockerfile and docker-compose.yml that serves as an example for the basics of Docker and Docker Compose. The sections below provide an annotated view of the docker files, as well as a short guide on how to use them.

Note: The Dockerfile and docker-compose.yml are not finished, and you will need to use what you learned through 


## Write your Dockerfile
The ``Dockerfile` is the recipe in which you will need to specify how to set up the virtual environment for our example program. 

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



## 2) Build your local image
Building a local image is fairly easy, and although it may be somewhat time-consuming, it is a set-and-forget process. To build the example image presented here, simply navigate to this (docker/) folder and run

```
docker build . --tag "vortex:tutorial"
```
where `tag` will be the "human representation" of the hex tag, here set to *vortex:tutorial* as an example

## 3) Deploy your local image


## 4) Simplify the deployment process with docker-compose