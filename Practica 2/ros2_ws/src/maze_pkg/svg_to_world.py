#!/usr/bin/env python3
"""
svg_to_world.py  –  Convierte un SVG de mazegenerator.net a un mundo de Gazebo.

Uso:
    python3 svg_to_world.py maze.svg maze_custom.world

El script detecta las líneas del SVG y las convierte en paredes SDF de Gazebo.
"""

import sys
import xml.etree.ElementTree as ET
import math


def parse_svg_lines(svg_path: str):
    """Extrae todas las líneas del SVG."""
    tree = ET.parse(svg_path)
    root = tree.getroot()

    # Namespace del SVG
    ns = {'svg': 'http://www.w3.org/2000/svg'}

    lines = []

    # Buscar elementos <line>
    for line in root.iter('{http://www.w3.org/2000/svg}line'):
        x1 = float(line.get('x1', 0))
        y1 = float(line.get('y1', 0))
        x2 = float(line.get('x2', 0))
        y2 = float(line.get('y2', 0))
        lines.append((x1, y1, x2, y2))

    # Buscar elementos <rect> (algunos generadores usan rectángulos)
    for rect in root.iter('{http://www.w3.org/2000/svg}rect'):
        x = float(rect.get('x', 0))
        y = float(rect.get('y', 0))
        w = float(rect.get('width', 0))
        h = float(rect.get('height', 0))
        if w > 0 and h > 0:
            # Convertir rect a 4 líneas
            lines.append((x,     y,     x + w, y    ))
            lines.append((x + w, y,     x + w, y + h))
            lines.append((x + w, y + h, x,     y + h))
            lines.append((x,     y + h, x,     y    ))

    return lines


def svg_to_gazebo(lines, scale: float, wall_height: float, wall_thickness: float):
    """
    Convierte líneas SVG a modelos SDF de Gazebo.

    - scale          : metros por píxel SVG  (ej: 0.01 = 1cm/px)
    - wall_height    : altura de las paredes en metros
    - wall_thickness : grosor de las paredes en metros
    """
    models = []

    # Calcular el centro del SVG para centrar el mundo en (0,0)
    all_x = [l[0] for l in lines] + [l[2] for l in lines]
    all_y = [l[1] for l in lines] + [l[3] for l in lines]
    cx = (max(all_x) + min(all_x)) / 2.0
    cy = (max(all_y) + min(all_y)) / 2.0

    for i, (x1, y1, x2, y2) in enumerate(lines):
        # Longitud de la pared
        dx = (x2 - x1) * scale
        dy = (y2 - y1) * scale
        length = math.sqrt(dx**2 + dy**2)

        if length < 0.01:   # ignorar líneas degeneradas
            continue

        # Centro de la pared en coordenadas Gazebo
        # SVG: Y crece hacia abajo → invertir Y para Gazebo
        wx = ((x1 + x2) / 2.0 - cx) * scale
        wy = -((y1 + y2) / 2.0 - cy) * scale

        # Ángulo de rotación (yaw)
        yaw = math.atan2(dy, dx)  # ya con Y invertido no hace falta

        models.append({
            'name'     : f'wall_{i}',
            'x'        : wx,
            'y'        : wy,
            'z'        : wall_height / 2.0,
            'yaw'      : yaw,
            'length'   : length,
            'height'   : wall_height,
            'thickness': wall_thickness,
        })

    return models


def generate_world(models, output_path: str):
    """Genera el archivo .world de Gazebo en formato SDF."""

    wall_sdfs = []
    for m in models:
        sdf = f"""
    <model name="{m['name']}">
      <static>true</static>
      <pose>{m['x']:.4f} {m['y']:.4f} {m['z']:.4f} 0 0 {m['yaw']:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{m['length']:.4f} {m['thickness']:.4f} {m['height']:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{m['length']:.4f} {m['thickness']:.4f} {m['height']:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
        wall_sdfs.append(sdf)

    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_custom">

    <!-- Luz -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Suelo -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Física -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Paredes generadas desde SVG -->
{"".join(wall_sdfs)}

  </world>
</sdf>
"""

    with open(output_path, 'w') as f:
        f.write(world_content)

    print(f"World generado: {output_path}  ({len(models)} paredes)")


def main():
    if len(sys.argv) < 3:
        print("Uso: python3 svg_to_world.py <maze.svg> <output.world> [scale] [height] [thickness]")
        print("  scale     : metros por píxel  (default: 0.05)")
        print("  height    : altura de paredes (default: 0.25)")
        print("  thickness : grosor de paredes (default: 0.05)")
        sys.exit(1)

    svg_path    = sys.argv[1]
    output_path = sys.argv[2]
    scale       = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    height      = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
    thickness   = float(sys.argv[5]) if len(sys.argv) > 5 else 0.05

    print(f"Leyendo {svg_path}...")
    lines = parse_svg_lines(svg_path)
    print(f"  {len(lines)} líneas encontradas")

    models = svg_to_gazebo(lines, scale, height, thickness)
    print(f"  {len(models)} paredes generadas")

    generate_world(models, output_path)


if __name__ == '__main__':
    main()
