from setuptools import find_packages, setup

package_name = 'wombat_host'

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
    maintainer='Samuel Street',
    maintainer_email='planetaryeclipse@gmail.com',
    description='Host utilities for running the Wombat robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_controller_node = wombat_host.xbox_controller:main',
            'vision_visualizer_node = wombat_host.vision_visualizer:main'
        ],
    },
)
