from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


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
            executable='ballbot_sqp',
            name='ballbot_sqp',
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
