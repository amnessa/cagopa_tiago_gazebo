from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
import os
from ament_index_python.packages import get_package_share_directory
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

    return generate_moveit_rviz_launch(moveit_config)
