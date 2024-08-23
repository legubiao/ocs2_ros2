import shutil
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def is_wsl():
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False


def generate_launch_description():
    if is_wsl():
        prefix = "xterm -e"
        print("Current system is WSL, use xterm as terminal")
    elif shutil.which("terminator"):
        prefix = "terminator --new-tab -x"
        print("Terminator is installed, use terminator as terminal")
    else:
        prefix = "gnome-terminal --"
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
