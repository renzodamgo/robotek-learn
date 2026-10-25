import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rrc_lite_driver')
    config_file = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='',
                              description='serial port of the RRC Lite board; empty = /dev/rrc_lite or /dev/ttyACM0'),
        Node(
            package='rrc_lite_driver',
            executable='rrc_driver',
            name='rrc_driver',
            output='screen',
            parameters=[config_file, {'port': LaunchConfiguration('port')}]
        )
    ])
