import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'follow_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keny',
    maintainer_email='you@example.com',
    description='Follow controller dan serial bridge ke ESP32 untuk robot omniwheel',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'follow_controller_node = follow_control.follow_controller_node:main',
            'serial_bridge_node = follow_control.serial_bridge_node:main',
        ],
    },
)
