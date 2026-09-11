from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pacote_do_drone',
            executable='sensores',
            name='sensores_node',
            output='screen'
        )
    ])
