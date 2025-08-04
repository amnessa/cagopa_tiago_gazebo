#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # Get the package directories - using ur_description for UR3 robot
    pkg_ur_description = get_package_share_directory('ur_description')

    # Set the path to the URDF file for UR3
    urdf_file = os.path.join(pkg_ur_description, 'urdf', 'ur3.urdf.xacro')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true. Isaac Sim must also be using sim time.'),

        # Robot State Publisher
        # Reads /joint_states from Isaac Sim and publishes the robot's TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': use_sim_time
            }]
        ),

        # UR3 Controller Node
        # Subscribes to topics from Isaac Sim and publishes control commands
        Node(
            package='cagopa_tiago_gazebo',
            executable='ur3_controller_node',
            name='ur3_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
