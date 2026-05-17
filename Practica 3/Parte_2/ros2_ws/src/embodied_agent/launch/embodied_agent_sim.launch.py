"""Lanza Gazebo Classic 11 con el mundo tar_lab + TurtleBot3 (waffle) + los nodos del agente.

Usa Gazebo Classic en lugar de Ignition porque es mucho más estable bajo Docker
con GPUs AMD (no usa Ogre2). El robot simulado es un TB3 waffle, que tiene
cámara RGB y LIDAR — equivalentes funcionales a la OAK-D y RPLIDAR del TB4
real para el caso de uso del agente.

Topics que publica TB3 en sim (sin namespace):
  /camera/image_raw   (RGB)
  /scan               (LIDAR)
  /odom
  /cmd_vel            (entrada)
  /imu

Uso:
  ros2 launch embodied_agent embodied_agent_sim.launch.py
  ros2 launch embodied_agent embodied_agent_sim.launch.py world:=tar_lab x:=0 y:=0 yaw:=0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('embodied_agent')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    worlds_dir = os.path.join(pkg_share, 'worlds')
    default_world_path = os.path.join(worlds_dir, 'tar_lab.world')

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    # Para que Gazebo Classic encuentre los modelos de TB3 (ground_plane, etc.)
    tb3_models = os.path.join(pkg_tb3_gazebo, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=tb3_models + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='waffle'
    )

    # Arranca Gazebo Classic (gzserver + gzclient) con el mundo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'false'}.items(),
    )

    # Spawna el robot TB3 waffle
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x,
            'y_pose': y,
        }.items(),
    )

    # robot_state_publisher para que tf tenga la TF tree del robot
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # Nodos del agente. En TB3 sim los topics NO llevan namespace, así que el
    # brain_node se suscribe directamente a /camera/image_raw, /scan, etc. — no
    # hace falta remap.
    agent_nodes = [
        Node(
            package='embodied_agent', executable='audio_in_node',
            name='audio_in_node', output='screen',
        ),
        Node(
            package='embodied_agent', executable='brain_node',
            name='brain_node', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='embodied_agent', executable='speech_node',
            name='speech_node', output='screen',
        ),
        Node(
            package='embodied_agent', executable='action_executor_node',
            name='action_executor_node', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world_path,
                              description='Path absoluto al .world (default: tar_lab)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        set_gazebo_model_path,
        set_tb3_model,
        gazebo,
        robot_state_publisher,
        spawn_tb3,
        *agent_nodes,
    ])
