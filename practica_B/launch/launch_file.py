from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="practica_B",
                executable="person_recon",
            ),
            Node(
                package="practica_B",
                executable="camera_node",
            ),
            Node(
                package="practica_B",
                executable="speed_monitor",
            ),
        ]
    )
