import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_dir = get_package_share_directory('yahboomcar_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'MicroROS.urdf')
    install_dir = get_package_share_directory('yahboomcar_description').replace('/share/yahboomcar_description', '')
    gazebo_model_path = os.path.join(install_dir, 'share')

    # Ruta a los mundos que vienen instalados de fábrica en Gazebo 11
    world_path = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'worlds',
        PythonExpression(["'", LaunchConfiguration('world'), ".world'"])
    ])

    # 1. Cerebro
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file]), 'use_sim_time': True}]
    )

    # 2. Publicador de Articulaciones
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Simulador Gazebo (Ahora le pasamos el world_path)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 4. Invocar robot (Aparecerá a los 10 segundos)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'microros_pi5', '-z', '0.5'],
        output='screen'
    )
    
    delayed_spawn = TimerAction(period=40.0, actions=[spawn_entity])

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_model_path),
        DeclareLaunchArgument('world', default_value='maze_2',
                          description='World'),
        rsp_node,
        jsp_node,
        gazebo,
        delayed_spawn
    ])