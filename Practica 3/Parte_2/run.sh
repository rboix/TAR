#!/bin/bash
# Lanza el contenedor Docker para el embodied agent (Practica 3 Parte 2)
# Monta el workspace local y reenvía X11 para GUI (rviz, etc.)

# Carga variables del .env si existe
if [ -f "$(dirname "$0")/.env" ]; then
    set -a
    source "$(dirname "$0")/.env"
    set +a
fi

export containerName=embodied_agent

sleep 1 && \
    xhost +local:$(docker inspect --format='{{ .Config.Hostname }}' $containerName 2>/dev/null) >/dev/null 2>&1 &

docker run --rm -it \
    -e DISPLAY=${DISPLAY} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    --workdir="/workspace" \
    --volume="$PWD:/workspace:rw" \
    -e "TERM=xterm-256color" \
    -e ELEVENLABS_API_KEY="${ELEVENLABS_API_KEY}" \
    -e ELEVENLABS_VOICE_ID="${ELEVENLABS_VOICE_ID}" \
    -e OPENAI_API_KEY="${OPENAI_API_KEY}" \
    --cap-add=SYS_TIME \
    --device /dev/snd \
    --group-add audio \
    -v /run/user/$(id -u)/pulse:/run/user/$(id -u)/pulse \
    -e PULSE_SERVER=unix:/run/user/$(id -u)/pulse/native \
    --name $containerName \
    embodied_agent:latest bash -c "sudo hwclock --hctosys 2>/dev/null || true; cd /workspace/ros2_ws && colcon build --symlink-install; exec bash"
