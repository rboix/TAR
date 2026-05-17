"""Lanza Ignition Gazebo Fortress con TurtleBot 4 + nodos del agente.

A diferencia del launch TB3+Gazebo Classic, este es fiel al robot real:
mismos topics, mismo namespace, misma cámara (OAK-D simulada). Usar este
launch para *iterar contra la integración real* antes de la demo.

Usa un mundo PREDEFINIDO de turtlebot4_simulator (depot/warehouse/maze) en
lugar de un mundo custom de primitivas. Los meshes de fábrica tienen UVs
correctas y evitan los crashes Ogre2 que vimos con SDFs con primitivas.

Si la GUI sigue petando por problemas Ogre2 + GPU AMD, las env vars
LIBGL_ALWAYS_SOFTWARE / OGRE_RTT_MODE caen a software rendering (lento
pero estable). Si aun así no abre la ventana, los sensores siguen
publicando: visualiza con `rqt_image_view`.

Topics que publica TB4 sim con namespace=turtlebot4:
  /turtlebot4/oakd/rgb/preview/image_raw
  /turtlebot4/oakd/stereo/image_raw
  /turtlebot4/scan
  /turtlebot4/odom
  /turtlebot4/cmd_vel

Uso:
  ros2 launch embodied_agent embodied_agent_sim_tb4.launch.py
  ros2 launch embodied_agent embodied_agent_sim_tb4.launch.py world:=maze
  ros2 launch embodied_agent embodied_agent_sim_tb4.launch.py software_rendering:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    tb4_sim_share = get_package_share_directory('turtlebot4_ignition_bringup')

    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_rviz = LaunchConfiguration('rviz')
    software_rendering = LaunchConfiguration('software_rendering')

    # Workaround AMD GPU: render por software (solo si lo pides). Lento pero
    # estable. Si tu GPU se entiende bien con Ogre2 deja software_rendering=false
    # para tener performance nativa.
    set_libgl_sw = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE', value='1',
        condition=IfCondition(software_rendering),
    )
    set_ogre_rtt = SetEnvironmentVariable(
        name='OGRE_RTT_MODE', value='Copy',
        condition=IfCondition(software_rendering),
    )
    set_qt_sw = SetEnvironmentVariable(
        name='QT_QUICK_BACKEND', value='software',
        condition=IfCondition(software_rendering),
    )

    tb4_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_sim_share, 'launch', 'turtlebot4_ignition.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'namespace': namespace,
            'x': x,
            'y': y,
            'yaw': yaw,
            'rviz': use_rviz,
        }.items(),
    )

    agent_nodes = [
        Node(
            package='embodied_agent', executable='audio_in_node',
            name='audio_in_node', output='screen',
        ),
        Node(
            package='embodied_agent', executable='brain_node',
            name='brain_node', output='screen',
            parameters=[{'robot_ns': namespace, 'use_sim_time': True}],
            remappings=[
                ('/camera/image_raw', ['/', namespace, '/oakd/rgb/preview/image_raw']),
                ('/camera/depth',     ['/', namespace, '/oakd/stereo/image_raw']),
                ('/scan',             ['/', namespace, '/scan']),
                ('/odom',             ['/', namespace, '/odom']),
            ],
        ),
        Node(
            package='embodied_agent', executable='speech_node',
            name='speech_node', output='screen',
        ),
        Node(
            package='embodied_agent', executable='action_executor_node',
            name='action_executor_node', output='screen',
            parameters=[{'robot_ns': namespace, 'use_sim_time': True}],
            remappings=[
                ('/cmd_vel',          ['/', namespace, '/cmd_vel']),
                ('/navigate_to_pose', ['/', namespace, '/navigate_to_pose']),
            ],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Mundo predefinido: warehouse | depot | maze'),
        DeclareLaunchArgument('namespace', default_value='turtlebot4'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('software_rendering', default_value='false',
                              description='Forzar render por CPU si GPU AMD crashea Ogre2'),

        set_libgl_sw,
        set_ogre_rtt,
        set_qt_sw,
        tb4_ignition,
        *agent_nodes,
    ])
