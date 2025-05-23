from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wombat_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Street',
    maintainer_email='planetaryeclipse@gmail.com',
    description='Navigational software for the Wombat robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive_node = wombat_nav.differential_drive:main',
            'waypoint_publisher = wombat_nav.waypoint_publisher:main',
            'object_centering = wombat_nav.object_centering:main',
            'vision_detection = wombat_nav.vision_detection:main',
            'vision_detection_calibrate_color = wombat_nav.vision_detection_calibrate_color:main',
            'fake_odometry = wombat_nav.fake_odometry:main',
            'odom_listener = wombat_nav.odom_listener:main',
            'main_waypoint_publisher = wombat_nav.main_waypoint_publisher:main',
            'branch_waypoint_publisher = wombat_nav.branch_waypoint_publisher:main',
            'inverse_kinematics_controller = wombat_nav.inverse_kinematics_controller:main',
            'test1_waypoints = wombat_nav.test1_waypoints:main',
            'test2_waypoints = wombat_nav.test2_waypoints:main',
            'waypoint_action_server = wombat_nav.waypoint_action_server:main',
            'turn_action_server = wombat_nav.turn_action_server:main',
            'weeds_scan_action_server = wombat_nav.weeds_scan_action_server:main',
            'yolo_vision_detection = wombat_nav.yolo_vision_detection:main',
            'weeds_spray_action_server = wombat_nav.weeds_spray_action_server:main'
        ],
    },
)
