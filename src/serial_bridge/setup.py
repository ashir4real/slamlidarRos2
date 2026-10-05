from setuptools import setup
import os
from glob import glob

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airon',
    maintainer_email='your_email@example.com',
    description='Serial bridge for ROS2 and Arduino',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = serial_bridge.serial_bridge_node:main',
        ],
    },
)
