from setuptools import find_packages, setup

package_name = 'wombat_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aimen',
    maintainer_email='a9awan@uwaterloo.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_velocity_node = wombat_drivers.wheel_velocity_node:main',
            'open_loop_motor_driver_node = wombat_drivers.open_loop_motor_driver:main',
            'camera_driver_node = wombat_drivers.camera_driver:main',
            'stereo_camera_driver_node = wombat_drivers.stereo_camera_driver:main',
            'bno085_driver_node = wombat_drivers.bno085_driver:main',
            'stm32_motor_driver_node = wombat_drivers.stm32_motor_driver:main',
            'teseo_liv3f_driver_node = wombat_drivers.teseo_liv3f_driver:main'
        ],
    },
)
