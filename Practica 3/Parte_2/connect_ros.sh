#!/bin/bash
# Conecta una nueva terminal al contenedor embodied_agent ya en ejecución
docker exec -it embodied_agent bash -c "ulimit -c 0; cd /workspace/ros2_ws && colcon build --symlink-install; exec bash"
