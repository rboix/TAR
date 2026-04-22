#!/bin/bash

# 1. Nombre del contenedor e imagen
export containerName=ros_humble
export imageName=ros_humble:latest

# 2. Permisos X11 para Mac (Ejecutar en el host)
# Esto permite que Docker se comunique con XQuartz
xhost +localhost > /dev/null 2>&1

# 3. Docker Run
docker run --rm -it \
        --platform linux/amd64 \
        --name $containerName \
        --network host \
        --workdir="/workspace" \
        --volume="$PWD:/workspace:rw" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=host.docker.internal:0 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e LIBGL_ALWAYS_INDIRECT=0 \
        -e GALLIUM_DRIVER=llvmpipe \
        -e QT_X11_NO_MITSHM=1 \
        -e "TERM=xterm-256color" \
        $imageName bash
