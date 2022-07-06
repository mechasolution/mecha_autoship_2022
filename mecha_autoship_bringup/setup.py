import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'mecha_autoship_bringup'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob.glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max Cha',
    maintainer_email='max@mechasolution.com',
    description='ROS 2 launch script for starting the mechasolution autoship 2022 project',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecha_autoship_mcu_node = mecha_autoship_bringup.mecha_autoship_mcu_node:main',
            'mecha_autoship_lidar_node = mecha_autoship_bringup.mecha_autoship_lidar_node:main'
        ],
    },
)
