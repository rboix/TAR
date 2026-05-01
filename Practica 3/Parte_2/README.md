Dentro de la misma carpeta:

docker build --build-arg USERNAME=$USER --build-arg USER_UID=$(id -u) -t embodied_agent:latest .

colcon build --symlink-install
source install/setup.bash
ros2 launch embodied_agent embodied_agent.launch.py # Para robot real


