import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    # General arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ur10e", package_name="ur_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur_description"),
                "urdf",
                "ur.urdf.xacro",
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("ur_config"),
                "config",
                "ur10e.srdf"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("ur_config"), "config", "moveit_controllers.yaml"
            )
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("ur_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Static TF for world to base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    return [
        robot_state_publisher,
        rviz_node,
        move_group_node,
        static_tf,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Isaac Sim) clock if true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )