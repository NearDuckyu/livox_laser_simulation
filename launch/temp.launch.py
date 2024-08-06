#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    livox_pkg = get_package_share_directory('livox_laser_simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=os.path.join(livox_pkg, 'urdf', 'livox_avia.xacro'),
            description='path to urdf.xacro file to publish'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description':Command(['xacro',' ', LaunchConfiguration('xacro_path')])
            }]),
    ])