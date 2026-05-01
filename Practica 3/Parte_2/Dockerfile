FROM ubuntu:22.04

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

USER $USERNAME

RUN sudo apt-get update && sudo apt-get upgrade -y

RUN sudo apt-get install -y \
    git vim nano curl wget \
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
    libgl1-mesa-glx \
    portaudio19-dev \
    libasound2-dev \
    libpulse-dev \
    pulseaudio \
    alsa-utils \
    espeak \
    software-properties-common

RUN sudo add-apt-repository universe

# ROS 2 Humble
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt-get update && sudo apt-get upgrade -y
RUN sudo apt-get install -y ros-humble-desktop ros-dev-tools
RUN sudo apt-get install -y \
    ros-humble-turtlebot4-simulator \
    ros-humble-irobot-create-nodes \
    ros-humble-turtlebot4-navigation \
    ros-humble-nav2-bringup \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    python3-colcon-common-extensions

# Python deps for embodied agent
RUN pip3 install --upgrade pip
RUN pip3 install \
    google-genai \
    google-auth \
    openai \
    elevenlabs \
    gtts \
    pygame \
    sounddevice \
    pyaudio \
    numpy \
    opencv-python \
    Pillow \
    pydantic \
    pyyaml \
    requests \
    aiohttp

# PyTorch CPU-only (necesario para Whisper, versión GPU innecesaria en el PC de control)
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Whisper (local STT) — instalar después de torch
RUN pip3 install openai-whisper

# ROS 2 Python client
RUN pip3 install empy lark

# Source ROS 2 by default
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/$USERNAME/.bashrc
RUN echo "export PYTHONPATH=\$PYTHONPATH:/workspace/ros2_ws/install/embodied_agent/lib/python3.10/site-packages" >> /home/$USERNAME/.bashrc

USER $USERNAME
