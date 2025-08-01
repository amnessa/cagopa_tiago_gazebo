#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # Get the package directories
    pkg_tiago_description = get_package_share_directory('tiago_description')

    # Set the path to the URDF file
    urdf_file = os.path.join(pkg_tiago_description, 'urdf', 'tiago.urdf')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true. Isaac Sim must also be using sim time.'),

        DeclareLaunchArgument(
            'world',
            default_value='',
            description='World file path'),

        # Include TIAGo Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tiago_description, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world
            }.items()
        ),

        # Robot State Publisher
        # Reads /joint_states from Isaac Sim and publishes the robot's TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_file]),
                'use_sim_time': use_sim_time
            }]
        ),

        # TIAGo Controller Node
        # Subscribes to topics from Isaac Sim and publishes control commands
        Node(
            package='cagopa_tiago_gazebo',
            executable='tiago_controller_node',
            name='tiago_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
