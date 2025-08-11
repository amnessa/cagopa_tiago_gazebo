import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_config",
            description="Package containing robot description files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur10e.urdf.xacro",
            description="Name of the robot description file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "semantic_description_file",
            default_value="ur10e.srdf",
            description="Name of the semantic description file (SRDF).",
        )
    )

    # Get paths
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    semantic_description_file = LaunchConfiguration("semantic_description_file")

    # Get URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "config", description_file]
            ),
        ]
    )

    # Get SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="cat")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "config", semantic_description_file]
            ),
        ]
    )

    # Create a dictionary of parameters that we can pass to multiple nodes
    robot_description_parameters = {
        "robot_description": robot_description_content,
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        ),
    }

    # Define the nodes to launch
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_parameters],
    )

    jacobian_calculator_node = Node(
        package="cagopa_tiago_gazebo",
        executable="jacobian_calculator_node",
        output="screen",
        parameters=[robot_description_parameters],
    )

    ur10_velocity_controller_node = Node(
        package="cagopa_tiago_gazebo",
        executable="ur10_velocity_controller_node",
        output="screen",
    )

    nodes = [
        robot_state_publisher_node,
        jacobian_calculator_node,
        ur10_velocity_controller_node,
    ]

    return LaunchDescription(declared_arguments + nodes)