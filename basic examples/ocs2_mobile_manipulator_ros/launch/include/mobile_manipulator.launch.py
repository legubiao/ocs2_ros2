import os
import re
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def is_wsl():
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False


def generate_launch_description():
    prefix = "gnome-terminal --"
    if is_wsl():
        prefix = "xterm -e"
        print("Current system is WSL, use xterm as terminal")
    else:
        print("Current system is not WSL, use gnome-terminal as terminal")

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
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc',
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
            executable='mobile_manipulator_mpc_node',
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
