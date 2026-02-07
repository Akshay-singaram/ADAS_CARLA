"""Launch the MPC controller with its parameters."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_pkg = get_package_share_directory('space_arm_control')
    mpc_params = os.path.join(control_pkg, 'config', 'mpc_params.yaml')

    mpc_controller = Node(
        package='space_arm_control',
        executable='mpc_controller_node.py',
        name='mpc_controller',
        parameters=[mpc_params],
        output='screen',
    )

    return LaunchDescription([mpc_controller])
