## Ejecución en simulación

Arrancar el contenedor Docker (construye la imagen si no existe):

```bash
./run.sh
```

Dentro del contenedor, compilar y lanzar el mundo de simulación con la escena detective:

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
cd ..
./../launch_detective.sh
```

Esto lanza el mundo *warehouse* con el TurtleBot 4 y la escena del Caso A. Pasados ~25 s los objetos aparecen en el suelo delante del robot.

Eliminar el dock en otra terminal (necesario para que el robot pueda moverse):

```bash
ign service -s /world/warehouse/remove \
  --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "turtlebot4/standard_dock", type: MODEL'
```

---

## Ejecución con el robot real

### 1. Conectividad

1. Conectar el móvil por USB y activar *compartir conexión* (internet para el portátil).
2. Conectar el portátil a la WiFi del robot: red **Xiaomi Robot**, contraseña `turtlebot4`.

### 2. Arrancar el contenedor

```bash
./run.sh
```

### 3. Configurar el descubrimiento ROS 2

```bash
./configure_discovery.sh   # Discovery Server IP: 192.168.31.191
source ~/.bashrc
ros2 daemon stop; ros2 daemon start
```

Verificar que el robot responde:

```bash
ros2 topic list | grep turtlebot4   # deben aparecer topics del robot
```

### 4. Lanzar el proyecto

En una segunda terminal (dentro del contenedor):

```bash
./connect_ros.sh
ros2 launch embodied_agent embodied_agent.launch.py
```

### 5. Eliminar el dock

Igual que en simulación, el dock debe eliminarse o el robot no puede moverse:

```bash
ros2 action send_goal /turtlebot4/undock irobot_create_msgs/action/Undock "{}"
```

### 6. Parada de emergencia

Si el robot no para, ejecutar en una tercera terminal:

```bash
./connect_ros.sh
ros2 topic pub /estop std_msgs/msg/Empty "{}" --once
```
