"""
Full simulation launch: Gazebo + Arm + Controllers + MPC + Vision.

This is the main entry point for running the complete simulation.
Usage:
  ros2 launch space_arm_simulation full_simulation.launch.py
  ros2 launch space_arm_simulation full_simulation.launch.py headless:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('space_arm_simulation')
    control_pkg = get_package_share_directory('space_arm_control')
    vision_pkg = get_package_share_directory('space_arm_vision')

    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run without Gazebo GUI'
    )
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    # 1. Launch Gazebo with space world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # 2. Spawn the arm (with slight delay for Gazebo startup)
    spawn_arm = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_pkg, 'launch', 'spawn_arm.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ],
    )

    # 3. Launch MPC controller (delayed for controllers to initialize)
    mpc_control = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_pkg, 'launch', 'mpc_control.launch.py')
                ),
            ),
        ],
    )

    # 4. Launch vision pipeline (delayed for camera to start publishing)
    vision = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vision_pkg, 'launch', 'vision.launch.py')
                ),
            ),
        ],
    )

    return LaunchDescription([
        declare_headless,
        declare_sim_time,
        gazebo,
        spawn_arm,
        mpc_control,
        vision,
    ])
