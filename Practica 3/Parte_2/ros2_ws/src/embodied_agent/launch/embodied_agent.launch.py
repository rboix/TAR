from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value='turtlebot4',
        description='Namespace del robot real (turtlebot4) o vacío para simulación',
    )
    ns = LaunchConfiguration('robot_ns')

    return LaunchDescription([
        ns_arg,

        Node(
            package='embodied_agent',
            executable='audio_in_node',
            name='audio_in_node',
            output='screen',
        ),

        Node(
            package='embodied_agent',
            executable='brain_node',
            name='brain_node',
            output='screen',
            parameters=[{'robot_ns': ns}],
            # Remaps: suscripciones a los topics reales del TurtleBot 4
            remappings=[
                ('/camera/image_raw',  ['/', ns, '/oakd/rgb/preview/image_raw']),
                ('/camera/depth',      ['/', ns, '/oakd/stereo/image_raw']),
                ('/scan',              ['/', ns, '/scan']),
                ('/odom',              ['/', ns, '/odom']),
            ],
        ),

        Node(
            package='embodied_agent',
            executable='speech_node',
            name='speech_node',
            output='screen',
        ),

        Node(
            package='embodied_agent',
            executable='action_executor_node',
            name='action_executor_node',
            output='screen',
            parameters=[{'robot_ns': ns}],
            # Remaps: publicaciones de control al namespace del robot
            remappings=[
                ('/cmd_vel',           ['/', ns, '/cmd_vel']),
                ('/navigate_to_pose',  ['/', ns, '/navigate_to_pose']),
                ('/dock',              ['/', ns, '/dock']),
                ('/undock',            ['/', ns, '/undock']),
            ],
        ),
    ])
