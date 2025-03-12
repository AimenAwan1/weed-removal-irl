from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="wombat_drivers",
                executable="stm32_motor_driver_node",
            ),
            Node(
                package="wombat_drivers",
                executable="bno085_driver_node",
                name="bno085_driver_node"
            ),
            Node(
                package="wombat_nav",
                executable="vision_detection",
                name="vision_detection"
            ),
            Node(
                package="wombat_nav",
                executable="object_centering",
                name="object_centering"
            )
        ]
    )
