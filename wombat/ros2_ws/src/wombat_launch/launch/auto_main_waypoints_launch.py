import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vod_parameters = [
        {
            "frame_id": "camera_link",
            "subscribe_stereo": True,
            "subscribe_odom_info": True,
        }
    ]
    vod_remappings = [
        ("imu", "/imu/data"),
        ("left/image_rect", "/camera/infra1/image_rect_raw"),
        ("left/camera_info", "/camera/infra1/camera_info"),
        ("right/image_rect", "/camera/infra2/image_rect_raw"),
        ("right/camera_info", "/camera/infra2/camera_info"),
    ]

    return LaunchDescription(
        [
            # all low level drivers
            Node(
                package="wombat_drivers",
                executable="stm32_motor_driver_node",
            ),
            # Node(
            #     package="wombat_nav",
            #     executable="differential_drive_node",
            # ),
            Node(
                package="wombat_drivers",
                executable="bno085_driver_node",
            ),
            # visual odometry and mapping (taken primarily from the example
            # of using the real sense camera)
            SetParameter(name='depth_module.emitter_enabled', value=0),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("realsense2_camera"), "launch"
                        ),
                        "/rs_launch.py",
                    ]
                ),
                launch_arguments={
                    "camera_namespace": "",
                    "enable_gyro": "true",
                    "enable_accel": "true",
                    "enable_infra1": "true",
                    "enable_infra2": "true",
                    "enable_sync": "true",
                }.items(),
            ),
            Node(
                package="rtabmap_odom",
                executable="stereo_odometry",
                output="screen",
                parameters=vod_parameters,
                remappings=vod_remappings,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=vod_parameters,
                remappings=vod_remappings,
                arguments=["-d"],
            ),
            # for displaying the current estimated position of the robot frame
            Node(  # describes the transformation from the base_link to camera_link
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "-0.362282519917305",
                    "--z",
                    "-0.118221722886988",
                    "--pitch",
                    "-0.349065850398866",
                    "--frame-id",
                    "odom",
                    "--child-frame-id",
                    "base_link_odom",
                ],
            ),
            Node(  # describes the transformation from the base_link to camera_link
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "-0.362282519917305",
                    "--z",
                    "-0.118221722886988",
                    "--pitch",
                    "-0.349065850398866",
                    "--frame-id",
                    "camera_link",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            Node(
                package="wombat_nav",
                executable="odom_listener"
            ),
            Node(
                package="wombat_nav",
                executable="inverse_kinematics_controller",
                name="inverse_kinematics_controller"
            ),
            Node(
                package="wombat_nav",
                executable="main_waypoint_publisher",
                name="main_waypoint_publisher"
            )
            
        ]
    )
