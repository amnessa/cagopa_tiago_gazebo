from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Declare the launch argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    # Get the URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('ur_description'),
        'urdf',
        'ur.urdf.xacro'
    )

    # Process the xacro file to generate the robot description
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Get the ros2_controllers.yaml file path
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # Create the controller_manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
        output="screen",
        remappings=[
            ('/forward_position_controller/commands', '/isaac_joint_commands'),
            ('/joint_states', '/isaac_joint_states'),
        ]
    )

    # Create the joint_state_broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Create the joint_trajectory_controller spawner
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])