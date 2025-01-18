from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='wombat_drivers',
                executable='open_loop_motor_driver_node',
                name='open_loop_motor_driver_node'
            ),
            Node(
                package='wombat_nav',
                executable='differential_drive_node',
                name='differential_drive_node'
            )
        ]
    )
