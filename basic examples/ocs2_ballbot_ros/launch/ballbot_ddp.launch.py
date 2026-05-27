import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_LAUNCH_DIR, 'include'))
import ballbot_example as be  # noqa: E402


def generate_launch_description():
    prefix = os.environ.get("OCS2_TERMINAL_PREFIX", "xterm -e")
    params = be.node_parameters()

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        *be.declare_path_arguments(),
        be.visualize_launch(),
        Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_ddp',
            name='ballbot_ddp',
            parameters=params,
            output='screen',
        ),
        Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_dummy_test',
            name='ballbot_dummy_test',
            prefix=prefix,
            parameters=params,
            output='screen',
        ),
        Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_target',
            name='ballbot_target',
            prefix=prefix,
            output='screen',
        ),
    ])
