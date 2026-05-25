import os
import re
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


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
            name='urdfFile',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='taskFile',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='libFolder',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='enableJoystick',
            default_value='false',
            description='Whether to enable joystick control'
        ),
        DeclareLaunchArgument(
            name='enableAutoPosition',
            default_value='false',
            description='Whether to enable automatic marker position updates'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
            ),
            launch_arguments={
                'urdfFile': LaunchConfiguration('urdfFile'),
                'rviz': LaunchConfiguration('rviz')
            }.items()
        ),
        # SQP MPC Node (non-debug mode)
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_sqp_mpc_node',
            name='mobile_manipulator_mpc',
            condition=UnlessCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        # SQP MPC Node (debug mode with terminal)
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_sqp_mpc_node',
            name='mobile_manipulator_mpc',
            prefix=prefix,
            condition=IfCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_dummy_mrt_node',
            name='mobile_manipulator_dummy_mrt_node',
            prefix=prefix,
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_target',
            name='mobile_manipulator_target',
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'enableJoystick': LaunchConfiguration('enableJoystick')
                },
                {
                    'enableAutoPosition': LaunchConfiguration('enableAutoPosition')
                },
            ],
            output='screen',
        )
    ])
