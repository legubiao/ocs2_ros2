import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

_UTILS_DIR = os.path.join(
    get_package_share_directory('ocs2_ros_interfaces'), 'launch')
if _UTILS_DIR not in sys.path:
    sys.path.insert(0, _UTILS_DIR)

from ocs2_launch_utils import ocs2_codegen_dir, ocs2_task_file  # noqa: E402

_OCS2_PKG = 'ocs2_quadrotor'
_DEFAULT_CONFIG = 'mpc'


def generate_launch_description():
    prefix = os.environ.get("OCS2_TERMINAL_PREFIX", "xterm -e")
    node_params = [
        {'taskFile': LaunchConfiguration('taskFile')},
        {'libFolder': LaunchConfiguration('libFolder')},
    ]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('task_config', default_value=_DEFAULT_CONFIG),
        DeclareLaunchArgument(
            'taskFile',
            default_value=ocs2_task_file(_OCS2_PKG, _DEFAULT_CONFIG),
        ),
        DeclareLaunchArgument(
            'libFolder', default_value=ocs2_codegen_dir(_OCS2_PKG),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/visualize.launch.py']),
            launch_arguments={'use_joint_state_publisher': 'false'}.items(),
        ),
        Node(
            package='ocs2_quadrotor_ros',
            executable='quadrotor_mpc',
            name='quadrotor_mpc',
            parameters=node_params,
            output='screen',
        ),
        Node(
            package='ocs2_quadrotor_ros',
            executable='quadrotor_dummy_test',
            name='quadrotor_dummy_test',
            prefix=prefix,
            parameters=node_params,
            output='screen',
        ),
        Node(
            package='ocs2_quadrotor_ros',
            executable='quadrotor_target',
            name='quadrotor_target',
            prefix=prefix,
            output='screen',
        ),
    ])
