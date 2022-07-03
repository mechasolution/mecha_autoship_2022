import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mecha_autoship_lidar_param = LaunchConfiguration(
        'mecha_autoship_lidar_param',
        default=os.path.join(
        get_package_share_directory('mecha_autoship_bringup'),
        'param',
        'mecha_autoship_lidar.yaml'))

    return LaunchDescription([
        # MCU Node
        Node(
            package='mecha_autoship_bringup',
            executable='mecha_autoship_mcu_node',
            name='mecha_autoship_mcu_node',
            parameters=[],
            output='screen'),
        # IMU Filter (complementary filter')
        # Node(
        #     package='imu_complementary_filter',
        #     executable='complementary_filter_node',
        #     name='complementary_filter_node',
        #     parameters=[{
        #         'fixed_frame' : 'base_link',
        #         'use_mag' : True,
        #         'do_bias_estimation' : True,
        #         'do_adaptive_gain' : True,
        #         'gain_acc' : 0.01,
        #         'gain_mag' : 0.01,
        #         'publish_tf' : True
        #     }],
        #     output='screen'),

        # IMU Filter (madgwick filter)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            parameters=[{
                'gain': 0.1,
                # 'zeta': 0,
                'use_mag': True,
                'fixed_frame': 'base_link',
                'publish_tf': True
            }]
        ),

        # LiDAR
        DeclareLaunchArgument(
            'mecha_autoship_lidar_param',
            default_value=mecha_autoship_lidar_param
        ),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[mecha_autoship_lidar_param],
            namespace='/',
        ),

        # LiDAR 각도, 거리 -> 좌표 (m단위)
        Node(
            package='mecha_autoship_bringup',
            executable='mecha_autoship_lidar_node',
            name='mecha_autoship_lidar_node',
            parameters=[],
            output='screen'
        )
    ])