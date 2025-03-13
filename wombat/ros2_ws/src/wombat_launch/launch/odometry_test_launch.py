from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(  # describes the camera frame
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0.235", 
                "--y", "0", 
                "--z", "0.23", 
                "--roll", "-2.094395102393196", 
                "--pitch", "0", 
                "--yaw", "-1.570796326794897",                 
                "--frame-id", "base_link", 
                "--child-frame-id", "camera_link"]
        ),
        Node(
            package="rtabmap_odom",
            executable="stereo_odometry",
            output="screen",
            parameters=list({
                "frame_id": "camera_link",
                "subscribe_odom_info": True
            })
        ),
        Node(
            package="wombat_drivers",
            executable="stereo_camera_driver_both_images_node",
            arguments=[
                # "true"  # display_frames=True
            ],
            remappings=[
                ("/left/image_rect_color", "/left/image_rect"),
                ("/right/image_rect_color", "/right/image_rect")
            ]
        )
    ])