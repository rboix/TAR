"""Lanza el mundo warehouse de TurtleBot 4 + spawnea una escena detective.

Reutiliza el launch base (embodied_agent_sim_tb4.launch.py), espera a que
Ignition cargue del todo y luego inyecta los objetos del Caso A
("El derrame") a unos 2 m delante del robot vía
`ros2 run ros_gz_sim create`.

Escena por defecto (Caso A — sección 3 del CLAUDE.md):
  - Botella tumbada con tapón
  - Charco de agua al lado, derramado hacia un lado
  - 3 papeles dispersos (uno cerca del charco "mojado")
  - Mochila abierta apartada
  - Zapatilla suelta lejos de su pareja
  - Llavero caído (salió del bolsillo de la mochila al tirar de ella)
  - Móvil boca abajo (caído al levantarse corriendo)

Uso:
  ros2 launch embodied_agent embodied_agent_sim_tb4_detective.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


# (sdf_file, unique_name, x, y, z, roll, pitch, yaw)
# Coordenadas pensadas para warehouse con TB4 spawneado en (0,0,0,yaw=0).
# El robot mira hacia +X, así que la escena está a 1.5–2.5 m delante.
DETECTIVE_SCENE_CASE_A = [
    # Botella tumbada (pitch=π/2 ⇒ eje del cilindro a lo largo de X).
    ('bottle_spilled.sdf', 'evidence_bottle',  2.00,  0.00, 0.035, 0.0, 1.5708, 0.0),

    # Charco al lado de la botella, alargado hacia -Y (sugiere derrame
    # lateral). Está casi pegado al suelo.
    ('water_puddle.sdf',   'evidence_puddle',  2.00, -0.20, 0.003, 0.0, 0.0, 0.0),

    # Tres papeles. paper_2 está justo encima del charco ⇒ "mojado".
    ('paper.sdf',          'evidence_paper_1', 1.65,  0.30, 0.002, 0.0, 0.0,  0.20),
    ('paper.sdf',          'evidence_paper_2', 1.95, -0.25, 0.004, 0.0, 0.0, -0.50),
    ('paper.sdf',          'evidence_paper_3', 2.30,  0.45, 0.002, 0.0, 0.0,  1.10),

    # Mochila abierta a la derecha.
    ('backpack.sdf',       'evidence_backpack', 2.60, -0.70, 0.22, 0.0, 0.0, -0.70),

    # Zapatilla descolgada (la pareja se "perdió" en la huida).
    ('shoe.sdf',           'evidence_shoe',     1.30, -0.85, 0.05, 0.0, 0.0,  0.80),

    # Llavero caído en el suelo entre el charco y la mochila.
    # Narrativa: salió del bolsillo de la mochila cuando alguien la agarró con prisa.
    ('keys.sdf',           'evidence_keys',     2.25, -0.42, 0.003, 0.0, 0.0,  0.55),

    # Móvil caído boca abajo (R=π) cerca de los papeles.
    # Narrativa: se le cayó de la mano al levantarse corriendo de la silla.
    ('phone.sdf',          'evidence_phone',    1.82,  0.20, 0.005, 3.1416, 0.0, -0.35),
]


def _spawn_action(scene_dir: str, world_name: str, entry: tuple) -> ExecuteProcess:
    fname, name, x, y, z, roll, pitch, yaw = entry
    return ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', world_name,
            '-file', os.path.join(scene_dir, fname),
            '-name', name,
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw),
        ],
        output='screen',
    )


def generate_launch_description():
    share = get_package_share_directory('embodied_agent')
    base_launch_path = os.path.join(
        share, 'launch', 'embodied_agent_sim_tb4.launch.py')
    scene_dir = os.path.join(share, 'worlds', 'detective_scene')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        # Forzamos warehouse — la escena está calibrada para ese mundo.
        launch_arguments={'world': 'warehouse'}.items(),
    )

    spawn_actions = [
        _spawn_action(scene_dir, 'warehouse', entry)
        for entry in DETECTIVE_SCENE_CASE_A
    ]

    delayed_spawn = TimerAction(
        period=25.0,  # margen para que Ignition + TB4 estén plenamente listos
        actions=spawn_actions,
    )

    return LaunchDescription([base_launch, delayed_spawn])
