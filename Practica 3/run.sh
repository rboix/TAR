#!/bin/bash
# --rm -> will delete the container once it is stopped
# -it -> interactive tty terminal
# -v -> mount X11 socket for GUI applications
# -v -> mount local ./src directory to /home/ros-foxy/src in the container
# -e -> environment: set DISPLAY variable for X11 forwarding
# --network -> host: use the host's network stack
# --name -> ros: name the container 'ros'
# image -> run the 'my_ros_foxy' image

export containerName=ros_humble
sleep 3 && \
        xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerName` >/dev/null 2>&1 &

docker run --rm -it -e DISPLAY=${DISPLAY} \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --network host \
        --workdir="/workspace" \
        --volume="$PWD:/workspace:rw" -e "TERM=xterm-256color" \
        --volume="$PWD/yahboomcar_ws:/root/yahboomcar_ws" \
        --name $containerName \
        ros_humble:latest bash

# docker run --rm -it \
#     --gpus all \
#     --runtime=nvidia \
#     -e DISPLAY=$DISPLAY \
#     -e NVIDIA_VISIBLE_DEVICES=all \
#     -e NVIDIA_DRIVER_CAPABILITIES=all \
#     -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v /dev/dri:/dev/dri \
#     --network host \
#     --workdir="/workspace" \
#     --volume="$PWD:/workspace:rw" \
#     --name $containerName \
#     ros_humble:latest bash