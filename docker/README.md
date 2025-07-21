# 🐳 Docker Tutorial

This folder contains a Docker-based development environment tailored for a standard ROS 2 workspace. It’s designed not just to work — but to help you **understand** how it works. This tutorial will walk you through the building blocks of Docker in a learning-oriented, hands-on way.

---

## Prerequisites

Before getting started, make sure you have Docker installed:

- [Docker Desktop (macOS/Windows)](https://www.docker.com/products/docker-desktop)
- [Docker Engine (Linux)](https://docs.docker.com/engine/install/)
  - On Linux, prefix commands with `sudo`, or configure Docker as a non-root user:  
    👉 [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/)

---

## Expected Workspace Structure

This setup assumes your local file tree follows a standard ROS 2 workspace layout:

```bash
ros_ws/
├── build/
├── log/
├── install/
└── src/
    └── software-learning-period/
        └── docker/
            ├── Dockerfile
            ├── build.sh
            └── run.sh
```

---

## Step-by-Step Guide

### 1. Clone the Repository

If you haven't already:

```bash
git clone git@github.com:vortexntnu/software-learning-period.git
cd software-learning-period
```

### 2. Build the Docker Image

#### Ubuntu / Linux
```bash
cd docker
chmod +x build.sh
sudo ./build.sh
```
You may need to use sudo for Docker commands unless your user is added to the docker group.
See: [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/)

#### macOS
```bash
cd docker
chmod +x build.sh
./build.sh
```
On macOS, Docker Desktop handles permissions, so sudo is usually not needed.

**What this does**:
- Builds a Docker image using the Dockerfile in this directory.
- Tags the image as ```software-learning-period:latest```.

Inside the Dockerfile, you’ll find key instructions:
- ```FROM```: Defines the base image (ros:humble)
- ```COPY```: Copies your ROS workspace files
- ```CMD```: Defines the default startup command (bash)

### 3. Run the Container

#### Ubuntu / Linux
```bash
chmod +x run.sh
sudo ./run.sh
```

#### macOS
```bash
chmod +x build.sh
./build.sh
```

**What this does**:
- Starts a new interactive container from the image you built
- Mounts your ROS workspace from your host into /ros2_ws inside the container
- Opens a Bash shell so you can start working with ROS 2 right away

### 4. Use ROS 2 Inside the Container
Once inside the container:
```bash
colcon build
ros2 launch my_package my_launch_file.launch.py
```

All build artifacts (```build/```, ```install/```, ```log/```) will remain on your host since the volume is mounted into the container.

## Docker Basics & Commands

Here are some helpful Docker commands you can reference:

| **Command**                  | **Description**                          |
|-----------------------------|------------------------------------------|
| `docker ps -a`              | List all running and stopped containers  |
| `docker images`             | Show all locally available images        |
| `docker build -t <tag> .`   | Build image from a Dockerfile            |
| `docker run -it <image>`    | Run container interactively              |
| `docker rm <container>`     | Remove a stopped container               |
| `docker rmi <image>`        | Remove an image                          |
| `docker image prune`        | Remove unused images                     |

---

## Task: Make Installed Packages Persistent

One of the first things you'll notice when working in a container is that anything you install manually (e.g. with `apt`) is lost once the container is stopped and removed. To make installed tools available permanently, you must **bake them into the image** using the Dockerfile.

Let's try it!

---

### Task: Add `curl` to Your Image

You’ll now modify the Dockerfile to install a common tool: `curl`. This is just an example — the same approach works for Python packages, ROS tools, or anything else you need.

1. Open the file: `docker/Dockerfile`
2. Add the following line just before the `COPY . .` line:
```dockerfile
RUN apt update && apt install -y curl
```

Your Dockerfile will now look like this:
```dockerfile
# ------------------------------------------------------------------------------
# Base Image
# ------------------------------------------------------------------------------
ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE}

# ------------------------------------------------------------------------------
# Runtime Configuration
# ------------------------------------------------------------------------------
USER root
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------------------------
# Workspace Setup
# ------------------------------------------------------------------------------
ENV WORKSPACE=/ros2_ws
WORKDIR ${WORKSPACE}

# ------------------------------------------------------------------------------
# Install system dependencies
# ------------------------------------------------------------------------------
RUN apt update && apt install -y curl

# ------------------------------------------------------------------------------
# Copy Workspace Files
# ------------------------------------------------------------------------------
COPY . .

# ------------------------------------------------------------------------------
# Default Startup Command
# ------------------------------------------------------------------------------
CMD ["bash"]
```

3. Rebuild the image
```bash
./build.sh
```

4. Run the container:
```bash
./run.sh
```

5. Inside the container, verify:
```bash
curl --version
```

You’ve now added a dependency to the image itself — no need to reinstall it every time you start a new container!

```vbnet
Tip: Whenever you find yourself installing something manually inside the container, ask yourself: Should this go in the Dockerfile instead?
```

---

## Wrapping Up

You now have a working introduction to using Docker with a ROS 2 workspace — complete with:

- A preconfigured Dockerfile
- Build and run scripts
- Workspace volume mounting
- Persistent image customization

This setup serves as a **reusable starting point** for other ROS 2 projects. Feel free to **copy the entire `docker/` folder** into other repositories and adapt it to fit your needs.