from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Default to xterm because it is the most portable choice across desktop
    # Linux, WSL, and container environments (e.g. distrobox), where
    # gnome-terminal is typically unavailable. Override via OCS2_TERMINAL_PREFIX,
    # e.g. `export OCS2_TERMINAL_PREFIX="gnome-terminal --"`.
    prefix = os.environ.get("OCS2_TERMINAL_PREFIX", "xterm -e")

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='task_name',
            default_value='mpc'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/visualize.launch.py']),
            launch_arguments={
                'use_joint_state_publisher': 'false'
            }.items()
        ),
        Node(
            package='ocs2_cartpole_ros',
            executable='cartpole_mpc',
            name='cartpole_mpc',
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        ),
        Node(
            package='ocs2_cartpole_ros',
            executable='cartpole_dummy_test',
            name='cartpole_dummy_test',
            arguments=[LaunchConfiguration('task_name')],
            prefix=prefix,
            output='screen'
        )
    ])
