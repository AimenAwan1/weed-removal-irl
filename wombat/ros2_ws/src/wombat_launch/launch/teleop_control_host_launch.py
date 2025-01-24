from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="joy", executable="joy_node", name="joy_node"),
            Node(
                package="wombat_host",
                executable="xbox_controller_node",
                name="xbox_controller_node",
            ),
            Node(
                package="wombat_host",
                executable="vision_visualizer_node",
                name="vision_visualizer_node",
            ),
        ]
    )
