"""Spawn the space arm into a running Gazebo simulation."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg = get_package_share_directory('space_arm_description')
    control_pkg = get_package_share_directory('space_arm_control')

    xacro_file = os.path.join(description_pkg, 'urdf', 'space_arm.urdf.xacro')
    controllers_yaml = os.path.join(control_pkg, 'config', 'controllers.yaml')
    rviz_config = os.path.join(description_pkg, 'rviz', 'display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'space_arm',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    # Controller manager spawners (delayed to wait for Gazebo)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawning to ensure Gazebo is ready
    delayed_joint_state = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_effort = TimerAction(
        period=5.0,
        actions=[effort_controller_spawner],
    )

    # RViz (optional, useful with VNC)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_sim_time,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state,
        delayed_effort,
        rviz,
    ])
