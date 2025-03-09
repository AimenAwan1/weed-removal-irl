from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # need to declare launch arguments before using them below
        DeclareLaunchArgument(
            name='approximate_sync', default_value='False',
            description='Whether to use approximate synchronization of topics. Set to true if '
                        'the left and right cameras do not produce exactly synced timestamps.'
        ),
        DeclareLaunchArgument(
            name='avoid_point_cloud_padding', default_value='False',
            description='Avoid alignment padding in the generated point cloud.'
                        'This reduces bandwidth requirements, as the point cloud size is halved.'
                        'Using point clouds without alignment padding might degrade performance '
                        'for some algorithms.'
        ),
        DeclareLaunchArgument(
            name='use_color', default_value='False',
            description='Generate point cloud with rgb data.'
        ),
        DeclareLaunchArgument(
            name='launch_image_proc', default_value='True',
            description='Whether to launch debayer and rectify nodes from image_proc.'
        ),
        DeclareLaunchArgument(
            name='namespace', default_value='',
            description='Namespace for all components loaded'
        ),
        DeclareLaunchArgument(
            name='left_namespace', default_value='left',
            description='Namespace for the left camera'
        ),
        DeclareLaunchArgument(
            name='right_namespace', default_value='right',
            description='Namespace for the right camera'
        ),
        DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created.'
            )
        ),
        # Stereo algorithm parameters
        DeclareLaunchArgument(
            name='stereo_algorithm', default_value='0',
            description='Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1)'
        ),
        DeclareLaunchArgument(
            name='prefilter_size', default_value='9',
            description='Normalization window size in pixels (must be odd)'
        ),
        DeclareLaunchArgument(
            name='prefilter_cap', default_value='31',
            description='Bound on normalized pixel values'
        ),
        DeclareLaunchArgument(
            name='correlation_window_size', default_value='15',
            description='SAD correlation window width in pixels (must be odd)'
        ),
        DeclareLaunchArgument(
            name='min_disparity', default_value='0',
            description='Disparity to begin search at in pixels'
        ),
        DeclareLaunchArgument(
            name='disparity_range', default_value='64',
            description='Number of disparities to search in pixels (must be a multiple of 16)'
        ),
        DeclareLaunchArgument(
            name='texture_threshold', default_value='10',
            description='Filter out if SAD window response does not exceed texture threshold'
        ),
        DeclareLaunchArgument(
            name='speckle_size', default_value='100',
            description='Reject regions smaller than this size in pixels'
        ),
        DeclareLaunchArgument(
            name='speckle_range', default_value='4',
            description='Maximum allowed difference between detected disparities'
        ),
        DeclareLaunchArgument(
            name='disp12_max_diff', default_value='0',
            description='Maximum allowed difference in the left-right disparity check in pixels '
                        '(Semi-Global Block Matching only)'
        ),
        DeclareLaunchArgument(
            name='uniqueness_ratio', default_value='15.0',
            description='Filter out if best match does not sufficiently exceed the next-best match'
        ),
        DeclareLaunchArgument(
            name='P1', default_value='200.0',
            description='The first parameter ccontrolling the disparity smoothness '
                        '(Semi-Global Block Matching only)'
        ),
        DeclareLaunchArgument(
            name='P2', default_value='400.0',
            description='The second parameter ccontrolling the disparity smoothness '
                        '(Semi-Global Block Matching only)'
        ),
        DeclareLaunchArgument(
            name='sgbm_mode', default_value='0',
            description='The mode of the SGBM matcher to be used'
        ),

        # now using all the launch arguments
        # Node(
        #     package='stereo_image_proc',
        #     executable='disparity_node',
        #     namespace=LaunchConfiguration('namespace'),
        #     parameters=[{
        #         'approximate_sync': LaunchConfiguration('approximate_sync'),
        #         'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
        #         'prefilter_size': LaunchConfiguration('prefilter_size'),
        #         'prefilter_cap': LaunchConfiguration('prefilter_cap'),
        #         'correlation_window_size': LaunchConfiguration('correlation_window_size'),
        #         'min_disparity': LaunchConfiguration('min_disparity'),
        #         'disparity_range': LaunchConfiguration('disparity_range'),
        #         'texture_threshold': LaunchConfiguration('texture_threshold'),
        #         'speckle_size': LaunchConfiguration('speckle_size'),
        #         'speckle_range': LaunchConfiguration('speckle_range'),
        #         'disp12_max_diff': LaunchConfiguration('disp12_max_diff'),
        #         'uniqueness_ratio': LaunchConfiguration('uniqueness_ratio'),
        #         'P1': LaunchConfiguration('P1'),
        #         'P2': LaunchConfiguration('P2'),
        #         'sgbm_mode': LaunchConfiguration('sgbm_mode'),
        #     }],
        #     remappings=[
        #         ('left/image_rect', [LaunchConfiguration('left_namespace'), '/image_rect']),
        #         ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
        #         ('right/image_rect', [LaunchConfiguration('right_namespace'), '/image_rect']),
        #         ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
        #     ]
        # ),
        Node(
            package='stereo_image_proc',
            executable='point_cloud_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'avoid_point_cloud_padding': LaunchConfiguration('avoid_point_cloud_padding'),
                'use_color': LaunchConfiguration('use_color'),
            }],
            remappings=[
                ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
                ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
                (
                    'left/image_rect_color',
                    [LaunchConfiguration('left_namespace'), '/image_rect_color']
                ),
                (
                    'right/image_rect_color',
                    [LaunchConfiguration('right_namespace'), '/image_rect_color']
                ),
            ]
        )
    ])
