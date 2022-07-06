#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # Joystick Driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[],
            output='screen'),

        # Joystick Node
        Node(
            package='mecha_autoship_teleop',
            executable='mecha_autoship_joystick_node',
            name='mecha_autoship_joystick_node',
            parameters=[],
            output='screen')
    ])