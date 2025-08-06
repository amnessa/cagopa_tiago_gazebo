from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get the path to the xacro file from the ur_config package
    xacro_file = os.path.join(
        get_package_share_directory("ur_config"), "config", "ur10e.urdf.xacro"
    )

    # Process the xacro file to generate the robot description XML
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Build the MoveIt configuration, passing the pre-processed robot description
    moveit_config = (
        MoveItConfigsBuilder("ur10e", package_name="ur_config")
        .robot_description(mappings=robot_description)
        .to_moveit_configs()
    )

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Generate the RViz launch description
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    # Return a LaunchDescription with both the robot_state_publisher and RViz
    return LaunchDescription([robot_state_publisher_node, rviz_launch])
