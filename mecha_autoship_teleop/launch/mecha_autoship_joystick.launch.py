#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node

NAMESPACE = os.environ.get('ROS_NAMESPACE', '')

def generate_launch_description():
    # 조이스틱 드라이버 노드
    joystick_driver_node = Node(
            namespace=NAMESPACE,
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[],
            output='screen'
        )
    # 조이스틱 데이터 가공 노드
    joystick_data_node = Node(
            namespace=NAMESPACE,
            package='mecha_autoship_teleop',
            executable='mecha_autoship_joystick_node',
            name='mecha_autoship_joystick_node',
            parameters=[],
            output='screen'
        )

    return LaunchDescription([
        joystick_driver_node,
        joystick_data_node
    ])