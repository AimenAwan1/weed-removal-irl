from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="wombat_drivers",
             executable="stereo_camera_driver_node"),        
    ])