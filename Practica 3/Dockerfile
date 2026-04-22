# Include a base image
FROM ubuntu:22.04

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)

ARG USERNAME
ARG USER_UID
ARG USER_GID=$USER_UID

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Madrid \
    DEBIAN_FRONTEND=noninteractive

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt upgrade -y

# Install Git, vim, and nano
RUN sudo apt-get install -y git vim nano

RUN sudo apt-get install -y \
    build-essential \
    python3-dev \
    python3-pip \
    python3-venv \
    libopenblas-dev \
    liblapack-dev \
    libjpeg-dev \
    libpng-dev \
    libprotobuf-dev \
    protobuf-compiler \
    ffmpeg \
    libsm6 \
    libxext6 \
    libgl1-mesa-glx

RUN sudo apt-get install software-properties-common -y
RUN sudo add-apt-repository universe

RUN sudo apt-get update && sudo apt-get install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt-get update && sudo apt-get upgrade -y

RUN sudo apt-get install ros-humble-desktop -y

RUN sudo apt-get install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes ros-humble-turtlebot4-navigation -y

RUN sudo apt-get install ros-dev-tools -y

RUN sudo apt-get update && sudo apt-get install wget -y
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install ignition-fortress -y

USER $USERNAME