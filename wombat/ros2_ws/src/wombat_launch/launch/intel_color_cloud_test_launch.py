import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    intel_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("realsense2_camera"), "launch"
                ),
                "/rs_launch.py",
            ]
        ),
        launch_arguments={"pointcloud.enable": "true"}.items(),
    )

    return LaunchDescription([intel_realsense])
