"""Launch visual servoing and target detection nodes."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vision_pkg = get_package_share_directory('space_arm_vision')
    params = os.path.join(vision_pkg, 'config', 'vision_params.yaml')

    target_detector = Node(
        package='space_arm_vision',
        executable='target_detector_node.py',
        name='target_detector',
        parameters=[params],
        output='screen',
    )

    visual_servoing = Node(
        package='space_arm_vision',
        executable='visual_servoing_node.py',
        name='visual_servoing',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([
        target_detector,
        visual_servoing,
    ])
