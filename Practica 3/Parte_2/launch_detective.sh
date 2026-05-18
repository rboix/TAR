#!/usr/bin/env bash
# Wrapper que lanza el detective y filtra el ruido del TurtleBot 4 sim.
#
# Los warnings que silenciamos vienen del nodo motion_control de Create 3
# y se publican vía console_bridge (no son loggers ROS 2 estándar, así que
# --log-level no los afecta). Son inocuos: motion_control descarta TFs
# "del pasado" pero sigue funcionando con la más reciente válida.
#
# Uso:
#   ./launch_detective.sh                   # caso A (warehouse + escena)
#   ./launch_detective.sh sim_tb4           # solo warehouse, sin escena
#   ./launch_detective.sh sim               # mundo TB3 antiguo
#   FILTER=0 ./launch_detective.sh          # desactiva el filtro
#
# Para ver TODO sin filtro:  FILTER=0 ./launch_detective.sh

set -u

LAUNCH_TARGET="${1:-sim_tb4_detective}"

# Patrones a tragar. Si necesitas ver alguno, quítalo de aquí.
SUPPRESS='TF_OLD_DATA|ignoring data from the past for frame (left_wheel|right_wheel|base_link)|Authority undetectable|buffer_core\.cpp|Possible reasons are listed at|FP16 is not supported on CPU'

CMD=(ros2 launch embodied_agent "embodied_agent_${LAUNCH_TARGET}.launch.py")
echo "[launch_detective] ${CMD[*]}"
echo "[launch_detective] filtrando: ${SUPPRESS}"
echo "[launch_detective] (pon FILTER=0 para ver todo el output)"
echo

if [[ "${FILTER:-1}" == "0" ]]; then
    exec "${CMD[@]}"
else
    # --line-buffered: que cada línea aparezca al instante, no en bloques.
    # 2>&1: motion_control suele escribir por stderr.
    # || true: que el exit code del pipeline sea el de ros2 launch, no el de grep.
    "${CMD[@]}" 2>&1 | grep --line-buffered -Ev "$SUPPRESS"
fi
