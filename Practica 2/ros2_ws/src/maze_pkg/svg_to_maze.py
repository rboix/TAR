#!/usr/bin/env python3
"""
svg_to_maze.py  –  Convierte un SVG de mazegenerator.net a los tres archivos
                   necesarios para Gazebo, en el mismo formato que maze_1:

      <nombre>.world        →  worlds/
      model.sdf             →  models/<nombre>/
      model.config          →  models/<nombre>/

Uso:
    python3 svg_to_maze.py <maze.svg> <nombre> [scale] [wall_height] [wall_thickness]

Ejemplos:
    python3 svg_to_maze.py maze.svg maze_custom
    python3 svg_to_maze.py maze.svg maze_custom 0.05 2.5 0.15

Argumentos:
    maze.svg        : archivo SVG exportado desde mazegenerator.net
    nombre          : nombre base (se usará en los archivos y en Gazebo)
    scale           : metros por píxel SVG          (default: 0.05)
    wall_height     : altura de las paredes en m    (default: 2.5)
    wall_thickness  : grosor de las paredes en m    (default: 0.15)
"""

import sys
import os
import math
import xml.etree.ElementTree as ET


# ──────────────────────────────────────────────────────────────────────────────
#  1. Parsear el SVG
# ──────────────────────────────────────────────────────────────────────────────

def parse_svg_lines(svg_path: str):
    tree = ET.parse(svg_path)
    root = tree.getroot()
    lines = []
    for line in root.iter('{http://www.w3.org/2000/svg}line'):
        x1 = float(line.get('x1', 0))
        y1 = float(line.get('y1', 0))
        x2 = float(line.get('x2', 0))
        y2 = float(line.get('y2', 0))
        lines.append((x1, y1, x2, y2))
    return lines


# ──────────────────────────────────────────────────────────────────────────────
#  2. Convertir líneas SVG → datos de paredes Gazebo
# ──────────────────────────────────────────────────────────────────────────────

def compute_walls(lines, scale, wall_height, wall_thickness):
    all_x = [v for seg in lines for v in (seg[0], seg[2])]
    all_y = [v for seg in lines for v in (seg[1], seg[3])]
    cx = (max(all_x) + min(all_x)) / 2.0
    cy = (max(all_y) + min(all_y)) / 2.0

    walls = []
    for i, (x1, y1, x2, y2) in enumerate(lines):
        dx = (x2 - x1) * scale
        dy = (y2 - y1) * scale
        length = math.sqrt(dx**2 + dy**2)
        if length < 0.01:
            continue

        # Centro en coordenadas Gazebo (Y invertida respecto a SVG)
        wx  = ((x1 + x2) / 2.0 - cx) * scale
        wy  = -((y1 + y2) / 2.0 - cy) * scale
        yaw = math.atan2(-(y2 - y1), (x2 - x1))

        walls.append({
            'index'    : i,
            'x'        : wx,
            'y'        : wy,
            'yaw'      : yaw,
            'length'   : length,
            'height'   : wall_height,
            'thickness': wall_thickness,
        })
    return walls


# ──────────────────────────────────────────────────────────────────────────────
#  3. Generar el bloque SDF de links (compartido por .world y model.sdf)
# ──────────────────────────────────────────────────────────────────────────────

def wall_link_sdf(w, indent='    ') -> str:
    i  = w['index']
    h  = w['height']
    return f"""{indent}<link name='Wall_{i}'>
{indent}  <collision name='Wall_{i}_Collision'>
{indent}    <geometry>
{indent}      <box>
{indent}        <size>{w['length']:.4f} {w['thickness']:.4f} {h:.4f}</size>
{indent}      </box>
{indent}    </geometry>
{indent}    <pose>0 0 {h/2:.4f} 0 -0 0</pose>
{indent}    <max_contacts>10</max_contacts>
{indent}    <surface>
{indent}      <contact><ode/></contact>
{indent}      <bounce/>
{indent}      <friction><torsional><ode/></torsional><ode/></friction>
{indent}    </surface>
{indent}  </collision>
{indent}  <visual name='Wall_{i}_Visual'>
{indent}    <pose>0 0 {h/2:.4f} 0 -0 0</pose>
{indent}    <geometry>
{indent}      <box>
{indent}        <size>{w['length']:.4f} {w['thickness']:.4f} {h:.4f}</size>
{indent}      </box>
{indent}    </geometry>
{indent}    <material>
{indent}      <script>
{indent}        <uri>file://media/materials/scripts/gazebo.material</uri>
{indent}        <n>Gazebo/Grey</n>
{indent}      </script>
{indent}      <ambient>1 1 1 1</ambient>
{indent}    </material>
{indent}    <meta><layer>0</layer></meta>
{indent}  </visual>
{indent}  <pose>{w['x']:.6f} {w['y']:.6f} 0 0 -0 {w['yaw']:.5f}</pose>
{indent}  <self_collide>0</self_collide>
{indent}  <enable_wind>0</enable_wind>
{indent}  <kinematic>0</kinematic>
{indent}</link>"""


# ──────────────────────────────────────────────────────────────────────────────
#  4. Generar <nombre>.world   (mismo formato que maze_1.world)
# ──────────────────────────────────────────────────────────────────────────────

def generate_world(walls, name, output_path):
    links_str = '\n'.join(wall_link_sdf(w, indent='        ') for w in walls)

    content = f"""<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional><ode/></torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/Grey</n>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='{name}'>
      <model name='{name}'>
        <pose>0 0 0 0 -0 0</pose>
        <static>1</static>
{links_str}
      </model>
      <pose>0 0 0 0 -0 0</pose>
    </model>
  </world>
</sdf>
"""
    with open(output_path, 'w') as f:
        f.write(content)
    print(f"  [OK] {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
#  5. Generar model.sdf   (mismo formato que el model.sdf de maze_1)
# ──────────────────────────────────────────────────────────────────────────────

def generate_model_sdf(walls, name, output_path):
    links_str = '\n'.join(wall_link_sdf(w, indent='    ') for w in walls)

    content = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='{name}'>
    <pose>0 0 0 0 -0 0</pose>
    <static>1</static>
{links_str}
  </model>
</sdf>
"""
    with open(output_path, 'w') as f:
        f.write(content)
    print(f"  [OK] {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
#  6. Generar model.config   (mismo formato que el de maze_1)
# ──────────────────────────────────────────────────────────────────────────────

def generate_model_config(name, output_path):
    content = f"""<?xml version="1.0" ?>
<model>
    <name>{name}</name>
    <version>1.0</version>
    <sdf version="1.7">model.sdf</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>
"""
    with open(output_path, 'w') as f:
        f.write(content)
    print(f"  [OK] {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
#  7. Generar launch file
# ──────────────────────────────────────────────────────────────────────────────

def generate_launch(name, output_path):
    content = f"""#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose',       default='0.0')
    y_pose       = LaunchConfiguration('y_pose',        default='0.0')

    world = os.path.join(
        get_package_share_directory('maze_pkg'),
        'worlds',
        '{name}.world'
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={{'world': world}}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={{'use_sim_time': use_sim_time}}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={{
            'x_pose': x_pose,
            'y_pose': y_pose
        }}.items()
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    return ld
"""
    with open(output_path, 'w') as f:
        f.write(content)
    print(f"  [OK] {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    svg_path  = sys.argv[1]
    name      = sys.argv[2]
    scale     = float(sys.argv[3]) if len(sys.argv) > 3 else 0.13
    height    = float(sys.argv[4]) if len(sys.argv) > 4 else 2.5
    thickness = float(sys.argv[5]) if len(sys.argv) > 5 else 0.10

    # Crear carpetas de salida
    model_dir = os.path.join('models', name)
    os.makedirs('worlds',     exist_ok=True)
    os.makedirs('launch',     exist_ok=True)
    os.makedirs(model_dir,    exist_ok=True)

    print(f"\nProcesando '{svg_path}'  →  nombre='{name}'  scale={scale}")
    lines = parse_svg_lines(svg_path)
    print(f"  {len(lines)} líneas encontradas en el SVG")

    walls = compute_walls(lines, scale, height, thickness)
    print(f"  {len(walls)} paredes generadas\n")

    print("Generando archivos:")
    generate_world(      walls, name, os.path.join('worlds', f'{name}.world'))
    generate_model_sdf(  walls, name, os.path.join(model_dir, 'model.sdf'))
    generate_model_config(      name, os.path.join(model_dir, 'model.config'))
    generate_launch(            name, os.path.join('launch',  f'{name}.launch.py'))

    print(f"""
Listo. Copia las carpetas generadas a tu paquete:

    cp worlds/{name}.world          ros2_ws/src/maze_pkg/worlds/
    cp -r models/{name}/            ros2_ws/src/maze_pkg/models/
    cp launch/{name}.launch.py      ros2_ws/src/maze_pkg/launch/

Añade en setup.py de maze_pkg:
    (os.path.join('share', package_name, 'models', '{name}'),
        glob('models/{name}/*')),

Luego:
    cd ros2_ws && colcon build --symlink-install --packages-select maze_pkg
    source install/setup.bash
    export TURTLEBOT3_MODEL=waffle
    ros2 launch maze_pkg {name}.launch.py
""")


if __name__ == '__main__':
    main()
