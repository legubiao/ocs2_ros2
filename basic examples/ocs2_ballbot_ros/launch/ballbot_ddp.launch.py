import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


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
            package='ocs2_ballbot_ros',
            executable='ballbot_ddp',
            name='ballbot_ddp',
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        ),
        Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_dummy_test',
            name='ballbot_dummy_test',
            prefix=prefix,
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        ),
        Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_target',
            name='ballbot_target',
            prefix=prefix,
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        )
    ])
