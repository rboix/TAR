# Práctica 3: Explorando nuevas herramientas

## Índice
1. [Entrega](#entrega)
2. [Parte 1: SLAM Simulado](#parte-1-slam-simulado)
    1. [Ejercicios](#ejercicios)
3. [Parte 2: A TU AIRE](#parte-2-a-tu-aire)
    1. [Turtlebot 4](#turtlebot-4)
    2. [Yahboom Car](#yahboom-car)
    3. [Ejercicios](#ejercicios-1)

## Entrega
La entrega de esta práctica se realizará a través de la herramienta de 'Evaluación' de UaCloud. La misma debe ser una memoria en formato `.pdf` en la cual se encuentren las respuestas a las preguntas teóricas y a los ejercicios propuestos de las diferentes partes. Deberá tener el nombre de *Apellidos_Nombre.pdf*. Asimismo, esta práctica **se debe hacer en grupos de tres** (el nombre del `.pdf` debe ser el primer apellido e inicial de los integrantes y en la memoria también se deben añadir, Ej: `Ramirez_T_Pujol_F.pdf`). En esta práctica se deben entregar también los códigos que se hayan generado para resolver los ejercicios. Pueden o bien compartir un enlace al repositorio de `GitHub` que estén usando, o añadirnos como colaboradores (Nuestros usuarios de github son TamaiRamirezUA y bigpacopujol, aunque nos añadan como colaboradores, añadan el enlace al repositorio en la memoria) o compartartan enlace de `Drive`, como prefieran. Por otro lado, es **obligatorio** que graben la resolución de los ejercicios y los compartan en la memoria a través de un enlace también, así se puede observar la correcta ejecución de los ejercicios.

## Parte 1: SLAM Simulado

En esta parte se abordará la generación de mapas bidimensionales del entorno. Posteriormente, el mapa generado será empleado para la localización del robot. Para ello, se emplearán los paquetes predeterminados en el marco de trabajo ROS (Robot Operating System) para el proceso de mapeo.

El robot es el encargado de construir el mapa a medida que se desplaza. Se recomienda el uso de la teleoperación mediante teclado para el manejo del robot, ya que permite adaptar la velocidad de acuerdo a las necesidades del usuario y dirigir el robot de acuerdo a la dirección deseada. Es importante destacar que el robot no está capacitado para mapear áreas del entorno que no haya explorado previamente, y que la calidad del mapa se ve afectada negativamente por la celeridad con la que se desplaza el robot, con lo cual, se recomienda emplear una velocidad de movimiento moderada.

Previamente a la realización del mapeado de un entorno con el Turtlebot 3, se debe generar en primera instancia un paquete de ROS denominado `slam_pkg`. En este paquete se volcarán los ficheros y directorios (`launch`, `models` y `worlds`) que se encuentran en la carpeta `Parte_1` del presente repositorio.

Una vez compilado el paquete con los archivos y directorios mencionados, es necesario abrir múltiples terminales y ejecutar los siguientes comandos:

Terminal 1:
```bash
source /workspace/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch slam_pkg maze_2.launch.py world:=maze_2
```

Terminal 2:
```bash
source /workspace/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

Terminal 3:
```bash
source /workspace/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

Como se puede observar, a partir de la información del LiDAR, conforme el robot se desplaza por el entorno, va recopilando datos para generar un mapa del entorno. Una vez que el robot haya completado el proceso de mapeado, para garantizar la conservación de esta información, se debe almacenar dicho mapa en un fichero. Para ello, se debe ejecutar el siguiente comando en una nueva terminal dentro de la **raíz del paquete `slam_pkg`**:

Terminal 4: 
```bash
mkdir maps && cd maps
ros2 run nav2_map_server map_saver_cli -f <nombre_mapa>
```

> Pregunta 1: Analiza el archivo `.yaml` del mapa y explica que significa cada uno de los campos que se muestran.

Una vez que se ha obtenido el mapa, este se puede emplear para localizar al robot (determinar su ubicación en tiempo real) y para navegar por el entorno. Este procedimiento implica desplazarse del punto actual del robot a un destino específico, planificar la trayectoria óptima y evitar colisiones con obstáculos en el camino. En la interfaz de comandos de `rviz`, se han asignado dos botones para gestionar estas funciones:

- `2D pose estimate`: Permite marcar la posición (clic con el ratón) y la orientación (arrastrar el ratón) donde se encuentra ahora mismo el robot.  Esto es necesario para poder inicializar el algoritmo de localización que es un filtro de partículas. La nube de "flechitas" verdes representa las posiciones y orientaciones más plausibles para el robot en el instante actual, si el algoritmo funciona bien esta nube irá siguiendo la posición real del robot en todo momento, y cuanto mejor localizado esté el robot más "condensada" estará la nube.

- `2D Nav Goal`: Permite marcar el punto al que queremos que se mueva el robot. Primero tenemos que asegurarnos de que el robot está localizado (que la "nube" de flechitas verdes está en torno a la posición real). Si hay una trayectoria posible, aparecerá dibujada en rviz y el robot se irá moviendo por ella.

No obstante, estos botones no funcionarán si el mapa no está cargado en memoria y los nodos de ROS necesarios para la planificación de trayectorias y evitación de obstáculos:

Terminal 1:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch slam_pkg worlds.launch.py world:=maze_2
```

Terminal 2:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<ruta_al_map.yaml>
```

> Pregunta 2: Investiga qué significa esa especie de "recuadro de colores" (mapa de calor) que aparece rodeando al robot cuando se pone a calcular la trayectoria y se va moviendo ¿qué significan los colores cálidos/frios?
>
> Pregunta 3: Investiga qué algoritmo usa ROS2 por defecto para calcular la trayectoria hasta el destino. Explica su funcionamiento lo más intuitivamente que puedas en aprox. 100-150 palabras (no el código línea por línea sino la idea de cómo funciona).
>
> Pregunta 4: Averigua cuáles son esos nodos que necesitamos cargar en memoria para que funcione la navegación, pon los nombres y describe brevemente el papel de cada uno en 1-2 frases.

### Ejercicios:

1. Como puedes observar, en la carpeta `worlds` proporcionada en el directorio `Parte_1`, existe un world llamado `obstacules`. Prueba a mapear este entorno y responde a la siguiente pregunta:
    > Pregunta 2: ¿Observas diferencias en el mapeado respecto al primer entorno probado anteriormente?

2. Genera un entorno propio con obstáculos y con diferentes configuraciones. Asimismo, responde a la siguiente pregunta: 
    > Pregunta 3: ¿Crees que hay cierto tipo de entornos en los que funciona mejor? (espacios abiertos, espacios pequeños, pasillos,...)

3. Elige uno de los entornos, puede ser uno de los proporcionados o el que hayas generado en el ejercicio anterior y construye el mapa variando el parámetro `resolution` cuando lanzas `cartographer.launch.py`. Prueba al menos **5 valores** diferentes. 

    > Pregunta 5: ¿Cómo afecta este parámetro en la generación del mapa?

    Para poder repetir la misma prueba variando este parámetro, es posible grabar los datos de los sensores en un fichero `rosbag`. Este último puede ser reproducido posteriormente tantas veces como sea necesario, como si se tratara de información que el robot recibe en tiempo real. De esta manera, es posible repetir un experimento múltiples veces utilizando los mismos datos de entrada.

    Un `bag` es un formato de archivo para almacenar la información de los mensajes que se mandan. Estos ficheros se crean principalmente a través de la herramienta rosbag, que se suscribe a uno o más topics y almacena los mensajes de datos de forma consecutiva. Este fichero se usa para reproducir lo que ha ocurrido durante una experimentación y también para poder procesar los datos adquiridos, analizar o visualizar estos datos. 

    Parámetros:
    - record: Graba en un fichero bag el contenido de los topics especificados.
    - info: Muestra un resumen del contenido de un fichero bag.
    - play: Reproduce el contenido de uno o más ficheros bag.
    - check: Comprueba si el fichero bag es reproducible en el sistema actual o si puede ser migrado a otro sistema.
    - fix: Repara los mensajes en un fichero bag de forma que se pueda reproducir en el sistema actual.
    - filter: Convierte un fichero bag utilizando expresiones de Python. 
    - compress: Comprime uno o más ficheros bag.
    - decompress: Descomprime uno o más ficheros bag.
    - reindex: Reindexa uno o más ficheros bag que esten corruptos.

    Para poder emplear esta herramienta, lo conveniente es generar una carpeta dentro del paquete de ROS donde estemos trabajando. En una terminal nueva, ejecuta:
    ```bash
    mkdir bagfiles && cd bagfiles
    ```
    Investiga como usar esta herramienta para guardar y ejecutar los archivos `bag`. Asimismo, contesta a las siguientes preguntas:
    > Pregunta 6: ¿Qué información puedo extraer de un fichero bag?
    >
    > Pregunta 7: ¿Se puede modificar la velocidad de reproducción del archivo? ¿Cómo se puede modificar? ¿Afecta a la resolución del mapa generado?

4. En el siguiente [enlace](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) tienes la documentación de cómo lanzar el `Turtlebot 4` en su simulador gazebo propio (No hace falta hacer la instalación). Prueba a hacer SLAM con este robot en al menos dos de los entornos disponibles dentro de su paquetería. Documenta los pasos que has tenido que seguir para que el robot realice el SLAM. Asimismo, una vez realizado el mapeado, comanda al robot a moverse autonomamente por el mapa para alcanzar diferentes coordenadas que tú le indiques (Al menos 3 puntos) de forma continúa. Contesta a la siguiente pregunta:
    > Pregunta 8: ¿Qué diferencias notas respecto al funcionamiento del `Turtlebot 3`? Detalla si los `topics` son diferentes etc. 

5. A parte de los robots `Turtlebot` en sus diferentes versiones, esta vez tu objetivo es hacer SLAM con otro robot diferente, llamado `yahboomcar` tanto en el world `maze_2` como en `obstacles`. Asimismo, una vez realizado el mapeado, comanda al robot a moverse autonomamente por el mapa para alcanzar diferentes coordenadas que tú le indiques (Al menos 3 puntos) de forma continúa. Para ello debes descargar la carpeta `yahboomcar_ws` que se encuentra dentro de este [zip](https://drive.google.com/file/d/1FNS8N9pm16cNtxZLc9C_sQWE-Z2UH07c/view?usp=drive_link) y guardarla en la **raíz del directorio** donde ejecutes esta práctica. Asimismo, debes borrar los paquetes de ros `yahboomcar_description` y `yahboomcar_nav` y sustituirlos por los que se encuentran dentro de la carpeta `Parte 1` de este repositorio y compilarlos de forma independiente con `colcon build --packages-select <paquete>`. A continuación ejecuta los siguientes comandos para lanzar el robot:

    Terminal 1:
    ```bash
    sudo su
    cd ~/yahboomcar_ws
    source install/setup.bash
    ros2 launch yahboomcar_description gazebo_launch.py world:=maze_2
    ```

    Terminal 2:
    ```bash
    sudo su
    cd ~/yahboomcar_ws
    source install/setup.bash
    ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=true
    ```

    Terminal 3:
    ```bash
    sudo su
    cd ~/yahboomcar_ws
    source install/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

    Una vez guardado el mapa, para navegar por él con el robot ejecuta:
    Terminal 1:
    ```bash
    sudo su
    cd ~/yahboomcar_ws
    source install/setup.bash
    ros2 launch yahboomcar_description gazebo_launch.py world:=maze_2
    ```

    Terminal 2:
    ```bash
    sudo su
    cd ~/yahboomcar_ws
    source install/setup.bash
    ros2 launch yahboomcar_nav navigation_dwb_launch.py use_sim_time:=True map:=<ruta_al_map.yaml>
    ```

    Por último contesta a la siguiente pregunta:

    > Pregunta 9: ¿Qué diferencias observas respecto a los otros robots?

## Parte 2: A TU AIRE

En esta parte se trabajará con robots reales, concretamente con el `Turtlebot 4` y con el `Yahboom Car`. A diferencia de la parte anterior, en la que se seguía un procedimiento guiado para realizar SLAM y navegación, en este caso el objetivo principal consiste en plantear, diseñar e implementar una tarea propia empleando alguno de los robots disponibles.

La finalidad de esta parte es que el alumno explore las posibilidades que ofrecen los robots reales y desarrolle una aplicación robótica de forma más libre. Para ello, se podrán emplear los conocimientos adquiridos anteriormente sobre ROS 2, teleoperación, topics, launch files, sensores, navegación, percepción y visualización en `rviz`.

Algunas posibles tareas que se pueden desarrollar son las siguientes:

- Teleoperación con gestos.
- Seguimiento de objetos.
- Reconocimiento de objetos.
- Desplazamiento autónomo por un mapa.
- Patrullaje entre diferentes puntos del entorno.
- Detección y evitación de obstáculos.
- Integración de sensores externos.
- Cualquier otra aplicación que se considere interesante.

La idea es que se dé rienda suelta a la imaginación, siempre teniendo en cuenta las limitaciones físicas de los robots, la seguridad del entorno y el correcto funcionamiento de los nodos de ROS necesarios para cada tarea.

Antes de comenzar con el desarrollo de la aplicación propia, se proporciona a continuación una guía para poner en marcha los robots reales y realizar SLAM con ellos. Este procedimiento puede servir como punto de partida para comprobar que existe comunicación con el robot, que los topics se publican correctamente y que es posible visualizar y controlar el sistema desde el ordenador.

### Turtlebot 4

En primer lugar, es necesario conectarse a la red WiFi denominada `Xiaomi Robot`, cuya contraseña es: `turtlebot4`

Una vez conectado a la red, en el directorio `Parte 2` se encontrará el archivo `configure_discovery.sh`, que permite configurar el descubrimiento de nodos de ROS 2 para poder comunicarse correctamente con el robot real.

Es necesario abrir varias terminales y ejecutar los siguientes comandos.

Terminal 1:
```bash
./run.sh

# Ejecuta lo siguiente donde se debe mantener todo por defecto salvo:
# Discovery Server IP: 192.168.31.191
./configure_discovery.sh

source ~/.bashrc
ros2 daemon stop; ros2 daemon start
ros2 topic list
```

El comando `ros2 topic list` permite comprobar si el ordenador está recibiendo correctamente los topics publicados por el robot. Si la configuración se ha realizado correctamente, deberían aparecer los topics asociados al `Turtlebot 4`.

Una vez que se puedan visualizar los topics del robot, se debe ejecutar en la misma terminal el siguiente comando para lanzar el proceso de SLAM:

```bash
ros2 launch turtlebot4_navigation slam.launch.py namespace:=turtlebot4
```

Terminal 2:
```bash
./connect_ros.sh
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/turtlebot4
```

Esta terminal permite lanzar rviz para visualizar el robot, los datos de los sensores y el mapa que se va construyendo durante el proceso de SLAM.

Terminal 3:
```bash
./connect_ros.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtlebot4/cmd_vel
```

Esta última terminal permite teleoperar el robot mediante teclado. Es recomendable mover el robot lentamente, especialmente durante el proceso de mapeado, ya que movimientos bruscos o velocidades elevadas pueden producir errores en la construcción del mapa.

Como se puede observar, el `Turtlebot 4` utiliza un `namespace`, en este caso `/turtlebot4`, por lo que algunos topics no tienen el mismo nombre que en otros robots. Por ejemplo, para enviar velocidades al robot se debe remapear el topic `/cmd_vel` al topic `/turtlebot4/cmd_vel`.

A modo de curiosidad, para sacar el robot de su base de carga o hacer que vuelva a ella, se pueden emplear los siguientes comandos:

DOCK:
```bash
ros2 action send_goal /turtlebot4/dock irobot_create_msgs/action/Dock "{}"
```

UNDOCK:
```bash
ros2 action send_goal /turtlebot4/undock irobot_create_msgs/action/Undock "{}"
```

Para más información acerca de este robot consulta la [Documentación Oficial](https://turtlebot.github.io/turtlebot4-user-manual/)

### Yahboom Car

Al igual que en el caso anterior, para trabajar con el `Yahboom Car` es necesario conectarse previamente a la red WiFi denominada `Xiaomi Robot`, cuya contraseña es: `turtlebot4`

En este caso, la puesta en marcha del robot requiere conectarse por `ssh` a la raspberry del robot y ejecutar los nodos dentro de un contenedor Docker. Para ello, es necesario abrir varias terminales.

Terminal 1:
```bash
ssh pi@192.168.31.212 # Contraseña: yahboom
sh start_agent_rpi5.sh # Esperar a que cargue
```

A continuación, es obligatorio darle al boton de **RESET** del robot que se encuentra en la parte trasera del mismo, (Boton blanco).

Terminal 2:
```bash
ssh pi@192.168.31.212 # Contraseña: yahboom

docker ps # Observar el Container ID de la IMAGE: yahboomtechnology/ros-humble:4.1.2

# EJECUTA LO SIGUIENTE SOLO SI APARECE EL CONTENEDOR
docker exec -it <Container ID> /bin/bash

# SINO APARECE EL CONTENEDOR LANZALO CON
./ros2_humble.sh

ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

En esta terminal se accede al robot mediante `ssh`, se identifica el contenedor Docker que contiene el entorno de ROS 2 y se ejecuta el launch de puesta en marcha del robot.

Terminal 3:
```bash
./run.sh

sudo su
cd ~/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 topic list # Para comprobar que se ha ejecutado todo correctamente
```

El comando `ros2 topic list` permite comprobar si el ordenador es capaz de comunicarse correctamente con el robot. En ocasiones, los topics pueden tardar unos segundos en aparecer, por lo que se recomienda ejecutar este comando varias veces si inicialmente no se observa información.

Una vez que se puedan visualizar los topics del robot, se debe ejecutar en la misma terminal el siguiente comando para lanzar el proceso de SLAM:

```bash
ros2 launch yahboomcar_nav map_cartographer_launch.py
```

Terminal 4:
```bash
./connect.sh

sudo su
cd ~/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Esta terminal permite controlar el `Yahboom Car` mediante teclado. Al igual que en el caso del `Turtlebot 4`, es recomendable desplazar el robot lentamente y recorrer el entorno de forma ordenada para obtener un mapa lo más coherente posible.

En este robot es especialmente importante comprobar que el valor de `ROS_DOMAIN_ID` coincide entre los distintos terminales y dispositivos. Si este parámetro no se configura correctamente, los nodos de ROS 2 no podrán descubrirse entre sí y no aparecerán los topics esperados.

Para más información acerca de este robot consulta la [Documentación Oficial](https://github.com/YahboomTechnology/MicroROS-Car-Pi5)

### Ejercicios 
1. El objetivo principal de esta parte consiste en desarrollar una aplicación propia empleando uno de los robots reales disponibles: `Turtlebot 4` o `Yahboom Car`.

    La tarea elegida puede estar relacionada con navegación, percepción, interacción, teleoperación, control o cualquier otro aspecto de la robótica móvil. Algunas ideas posibles son:

    - Teleoperación con gestos.
    - Seguimiento de objetos mediante cámara.
    - Reconocimiento de objetos.
    - Desplazamiento autónomo por un mapa.
    - Movimiento entre varios puntos definidos por el usuario.
    - Detección de obstáculos y reacción ante ellos.
    - Exploración de una zona del entorno.
    - Comparación entre el comportamiento de dos robots.

    Antes de implementar la tarea, se debe describir brevemente qué se quiere conseguir, qué robot se va a utilizar y qué nodos, topics o paquetes de ROS 2 serán necesarios.

2. Implementa la aplicación propuesta en simulación. Para ello, se recomienda crear un paquete propio de ROS 2 donde se incluyan los nodos necesarios para resolver la tarea. En función de la tarea elegida, el nodo desarrollado podrá suscribirse a topics de sensores, publicar velocidades, enviar objetivos de navegación o procesar información procedente del robot.

3. Prueba la aplicación con el robot real. Durante las pruebas, presta atención al comportamiento del robot, a la estabilidad de la comunicación, a la frecuencia de publicación de los topics y a los posibles errores que puedan aparecer en los terminales. Si la aplicación implica movimiento, se debe garantizar en todo momento que el robot dispone de espacio suficiente para desplazarse y que existe la posibilidad de detenerlo rápidamente en caso necesario. 

    Documenta el procedimiento completo seguido para ejecutar tu aplicación. Deben indicarse claramente los comandos necesarios, los terminales empleados y el orden en el que deben lanzarse los nodos. La documentación debe ser suficientemente clara como para que otra persona pueda repetir el experimento empleando el mismo robot.
        
    > Pregunta 1: Describe los resultados obtenidos durante las pruebas. ¿El robot se comporta como esperabas? ¿Qué problemas han aparecido durante la ejecución?
    >
    > Pregunta 2: Incluye los comandos necesarios para ejecutar tu aplicación y explica brevemente qué se lanza en cada terminal.

4. Realiza una reflexión sobre las diferencias entre trabajar en simulación y trabajar con robots reales. Ten en cuenta aspectos como la comunicación, los sensores, el ruido, la batería, los tiempos de respuesta, los errores de localización, la seguridad y la dificultad de depuración.

    > Pregunta 3: ¿Qué diferencias principales has observado entre trabajar con un robot simulado y trabajar con un robot real? ¿Qué dificultades adicionales aparecen en el robot real?
