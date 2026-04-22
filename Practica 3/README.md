# Práctica 3: Explorando nuevas herramientas

## Entrega
La entrega de esta práctica se realizará a través de la herramienta de 'Evaluación' de UaCloud. La misma debe ser una memoria en formato `.pdf` en la cual se encuentren las respuestas a las preguntas teóricas y a los ejercicios propuestos de las diferentes partes. Deberá tener el nombre de *Apellidos_Nombre.pdf*. Asimismo, esta práctica **se debe hacer en grupos de tres** (el nombre del `.pdf` debe ser el primer apellido e inicial de los integrantes y en la memoria también se deben añadir, Ej: `Ramirez_T_Pujol_F.pdf`). En esta práctica se deben entregar también los códigos que se hayan generado para resolver los ejercicios. Pueden o bien compartir un enlace al repositorio de `GitHub` que estén usando, o añadirnos como colaboradores (Nuestros usuarios de github son TamaiRamirezUA y bigpacopujol, aunque nos añadan como colaboradores, añadan el enlace al repositorio en la memoria) o compartartan enlace de `Drive`, como prefieran. Por otro lado, es **obligatorio** que graben la resolución de los ejercicios y los compartan en la memoria a través de un enlace también, así se puede observar la correcta ejecución de los ejercicios.

## Parte 1: SLAM

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

    > Pregunta 4: ¿Cómo afecta este parámetro en la generación del mapa?

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
    > Pregunta 5: ¿Qué información puedo extraer de un fichero bag?
    >
    > Pregunta 6: ¿Se puede modificar la velocidad de reproducción del archivo? ¿Cómo se puede modificar? ¿Afecta a la resolución del mapa generado?

4. En el siguiente [enlace](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) tienes la documentación de cómo lanzar el `Turtlebot 4` en su simulador gazebo propio (No hace falta hacer la instalación). Prueba a hacer SLAM con este robot en al menos dos de los entornos disponibles dentro de su paquetería. Documenta los pasos que has tenido que seguir para que el robot realice el SLAM. Asimismo, una vez realizado el mapeado, comanda al robot a moverse autonomamente por el mapa para alcanzar diferentes coordenadas que tú le indiques (Al menos 3 puntos) de forma continúa. Contesta a la siguiente pregunta:
    > Pregunta 7: ¿Qué diferencias notas respecto al funcionamiento del `Turtlebot 3`? Detalla si los `topics` son diferentes etc. 

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


    