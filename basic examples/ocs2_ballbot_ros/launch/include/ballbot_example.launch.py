"""Shared launch fragments for ballbot basic example nodes."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_UTILS_DIR = os.path.join(
    get_package_share_directory('ocs2_ros_interfaces'), 'launch')
if _UTILS_DIR not in sys.path:
    sys.path.insert(0, _UTILS_DIR)

from ocs2_launch_utils import ocs2_codegen_dir, ocs2_task_file  # noqa: E402

_OCS2_PKG = 'ocs2_ballbot'
_DEFAULT_CONFIG = 'mpc'


def declare_path_arguments():
    return [
        DeclareLaunchArgument(
            'task_config', default_value=_DEFAULT_CONFIG,
            description='Config folder under share/config/ (e.g. mpc)',
        ),
        DeclareLaunchArgument(
            'taskFile',
            default_value=ocs2_task_file(_OCS2_PKG, _DEFAULT_CONFIG),
        ),
        DeclareLaunchArgument(
            'libFolder', default_value=ocs2_codegen_dir(_OCS2_PKG),
        ),
    ]


def node_parameters():
    return [
        {'taskFile': LaunchConfiguration('taskFile')},
        {'libFolder': LaunchConfiguration('libFolder')},
    ]


_LAUNCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def visualize_launch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_LAUNCH_DIR, 'visualize.launch.py')),
        launch_arguments={'use_joint_state_publisher': 'false'}.items(),
    )
