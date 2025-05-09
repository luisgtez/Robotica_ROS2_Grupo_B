from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='practica_B',
            executable='person_recon',
        ),
        Node(
            package='practica_B',
            executable='camera_node',
        )
    ])
