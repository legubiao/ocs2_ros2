import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


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
        DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/visualize.launch.py']),
            launch_arguments={
                'use_joint_state_publisher': 'false'
            }.items()
        ),
        Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_mpc',
            name='double_integrator_mpc',
            arguments=[LaunchConfiguration('task_name')],
            prefix=prefix,
            condition=IfCondition(LaunchConfiguration("debug")),
            output='screen'
        ),
        Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_mpc',
            name='double_integrator_mpc',
            arguments=[LaunchConfiguration('task_name')],
            condition=UnlessCondition(LaunchConfiguration("debug")),
            output='screen'
        ),
        Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_dummy_test',
            name='double_integrator_dummy_test',
            arguments=[LaunchConfiguration('task_name')],
            prefix=prefix,
            output='screen'
        ),
        Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_target',
            name='double_integrator_target',
            arguments=[LaunchConfiguration('task_name')],
            prefix=prefix,
            output='screen'
        )
    ])
