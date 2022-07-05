import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mecha_autoship_image_pub_params = LaunchConfiguration(
        'mecha_autoship_image_pub_params',
        default=os.path.join(
            get_package_share_directory('mecha_autoship_camera'),
            'param',
            'mecha_autoship_camera.yaml'
        )
    )

    mecha_autoship_image_color_filter_params = LaunchConfiguration(
        'mecha_autoship_image_color_filter_params',
        default=os.path.join(
            get_package_share_directory('mecha_autoship_camera'),
            'param',
            'mecha_autoship_camera.yaml'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mecha_autoship_image_pub_params',
            default_value=mecha_autoship_image_pub_params
        ),

        Node(
            package='mecha_autoship_camera',
            executable='mecha_autoship_image_pub_node',
            name='mecha_autoship_image_pub_node',
            output='screen',
            emulate_tty=True,
            parameters=[mecha_autoship_image_pub_params],
        ),

        DeclareLaunchArgument(
            'mecha_autoship_image_color_filter_params',
            default_value=mecha_autoship_image_color_filter_params
        ),

        Node(
            package='mecha_autoship_camera',
            executable='mecha_autoship_image_color_filter_node',
            name='mecha_autoship_image_color_filter_node',
            output='screen',
            emulate_tty=True,
            parameters=[mecha_autoship_image_color_filter_params],
        ),
    ])