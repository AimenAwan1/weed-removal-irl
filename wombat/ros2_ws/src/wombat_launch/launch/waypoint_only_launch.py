from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="meercat_if",
                namespace="meercat_if",
                executable="node",
                name="meercat_if",
            )
        ]
    )
