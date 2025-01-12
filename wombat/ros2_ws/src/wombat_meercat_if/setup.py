from setuptools import find_packages, setup

package_name = 'wombat_meercat_if'

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
    maintainer_email='sstreet@uwaterloo.ca',
    description='Node that interfaces with the external Meercat module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = wombat_meercat_if.meercat_if_node:main'
        ],
    },
)
