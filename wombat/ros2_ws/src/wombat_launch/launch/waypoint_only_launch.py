from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="wombat_meercat_if",
            #     namespace="meercat_if",
            #     executable="node",
            #     name="meercat_if",
            # ),
            Node(
                package='wombat_drivers',
                namespace='wombat_drivers',
                executable='open_loop_motor_driver_node',
                name='open_loop_motor_driver_node'
            ),
            Node(
                package='wombat_nav',
                namespace='wombat_nav',
                executable='differential_drive_node',
                name='differential_drive_node'
            )
        ]
    )
