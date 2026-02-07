"""Launch Gazebo with the space environment world."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('space_arm_simulation')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(sim_pkg, 'worlds', 'space_environment.world')

    # Headless mode for cloud environments
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI). Set true for Codespaces without VNC.'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    # Software rendering for cloud environments
    set_mesa = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items(),
    )

    # Gazebo client (GUI) -- skip if headless
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ),
    )

    return LaunchDescription([
        declare_headless,
        declare_sim_time,
        set_mesa,
        gzserver,
        gzclient,
    ])
