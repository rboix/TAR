FROM --platform=linux/amd64 ubuntu:22.04

ARG USERNAME
ARG USER_UID
ARG USER_GID=$USER_UID

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Madrid

# 1. Configurar Zona Horaria y Usuario (Como ROOT)
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y sudo \
    && (groupadd --gid $USER_GID $USERNAME || groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1)) \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

# 2. Instalar dependencias (Sigue como ROOT, sin sudo)
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y git vim nano curl wget software-properties-common
RUN add-apt-repository universe

# 3. ROS 2 Humble (Optimizado para ARM64)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-turtlebot4-simulator \
    ros-dev-tools

# 4. Gazebo Ignition
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list
RUN apt-get update && apt-get install -y ignition-fortress

# 5. Finalizar: Cambiar al usuario para el trabajo diario
USER $USERNAME
WORKDIR /home/$USERNAME