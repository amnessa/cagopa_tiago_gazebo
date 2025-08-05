from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur10_ball_tracker',
            executable='ball_tracker_node',
            name='ball_tracker'
        ),
        Node(
            package='ur10_ball_tracker',
            executable='jacobian_calculator_node',
            name='jacobian_calculator'
        ),
        Node(
            package='ur10_ball_tracker',
            executable='ur10_arm_controller_node',
            name='ur10_arm_controller'
        ),
        Node(
            package='ur10_ball_tracker',
            executable='mobile_base_controller',
            name='mobile_base_controller',
            prefix='xterm -e' # Launch in a new terminal
        ),
    ])
