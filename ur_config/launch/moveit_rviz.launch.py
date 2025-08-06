from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur10e", package_name="ur_config")
        .robot_description(file_path="config/ur10e.urdf.xacro")
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
