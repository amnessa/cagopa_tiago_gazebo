#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # Get the package directories - using ur_description for UR10 robot
    try:
        pkg_ur_description = get_package_share_directory('ur_description')
        # Set the path to the URDF file for UR10
        urdf_file = os.path.join(pkg_ur_description, 'urdf', 'ur10.urdf.xacro')
    except:
        # Fallback if ur_description is not available
        print("WARNING: ur_description package not found, using minimal URDF")
        urdf_file = None

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    launch_nodes = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true. Isaac Sim must also be using sim time.'),

        # UR10 Controller Node
        # Subscribes to /isaac_joint_states and publishes to /isaac_joint_commands
        Node(
            package='cagopa_tiago_gazebo',
            executable='ur10_controller_node',
            name='ur10_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]

    # Add robot state publisher only if URDF is available
    if urdf_file and os.path.exists(urdf_file):
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': use_sim_time
            }]
        )
        launch_nodes.insert(-1, robot_state_publisher_node)

    return LaunchDescription(launch_nodes)
