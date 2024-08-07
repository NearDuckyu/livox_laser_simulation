#!/usr/bin/env python3
 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # Package directories
    livox_pkg = get_package_share_directory('livox_laser_simulation')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    xacro_file=os.path.join(livox_pkg, 'urdf', 'livox_avia.xacro')
    robot_description = xacro.process_file(xacro_file)

    # Define launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(livox_pkg, 'worlds', 'standardrobots_factory.world')
    )

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false',
            'verbose': 'true'
        }.items()
    )

    # Robot State Publisher Node

    # Spawn URDF model Node
    

    # RViz Node
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(livox_pkg, 'rviz', 'livox_simulation.rviz')],
    #     output='screen'
    # )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description':robot_description.toxml()
        }]),
        Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'livox_lidar',
            '-topic', 'robot_description'
        ],
        output='screen'
        )
        # rviz
    ])
