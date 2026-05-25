import os

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _robot_state_publisher_node(context, *args, **kwargs):
    urdf_file = context.launch_configurations['urdfFile']
    robot_description = xacro.process_file(urdf_file).toxml()
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'publish_frequency': 100.0,
            'use_tf_static': True,
            'robot_description': robot_description,
        }],
    )]


def generate_launch_description():
    # Default to xterm because it is the most portable choice across desktop
    # Linux, WSL, and container environments (e.g. distrobox), where
    # gnome-terminal is typically unavailable. Override via OCS2_TERMINAL_PREFIX,
    # e.g. `export OCS2_TERMINAL_PREFIX="gnome-terminal --"`.
    prefix = os.environ.get("OCS2_TERMINAL_PREFIX", "xterm -e")

    rviz_config_file = get_package_share_directory('ocs2_legged_robot_ros') + "/rviz/legged_robot.rviz"

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='description_name',
            default_value='legged_robot_description'
        ),
        DeclareLaunchArgument(
            name='multiplot',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'ocs2_legged_robot') + '/config/mpc/task.info'
        ),
        DeclareLaunchArgument(
            name='referenceFile',
            default_value=get_package_share_directory(
                'ocs2_legged_robot') + '/config/command/reference.info'
        ),
        DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'ocs2_robotic_assets') + '/resources/anymal_c/urdf/anymal.urdf'
        ),
        DeclareLaunchArgument(
            name='gaitCommandFile',
            default_value=get_package_share_directory(
                'ocs2_legged_robot') + '/config/command/gait.info'
        ),
        DeclareLaunchArgument(
            name='raisimFile',
            default_value=get_package_share_directory(
                'ocs2_legged_robot_raisim') + '/config/raisim.info'
        ),
        DeclareLaunchArgument(
            name='resourcePath',
            default_value=get_package_share_directory(
                'ocs2_robotic_assets') + '/resources/anymal_c/meshes'
        ),
        DeclareLaunchArgument(
            name='policyFile',
            default_value=os.path.join(
                get_package_share_directory('ocs2_legged_robot_mpcnet'), 'policy', 'raisim.onnx')
        ),
        DeclareLaunchArgument(
            name='useRaisim',
            default_value='true'
        ),

        OpaqueFunction(function=_robot_state_publisher_node),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file],
            condition=IfCondition(
                LaunchConfiguration('rviz'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ocs2_legged_robot_ros'), 'launch', 'multiplot.launch.py')),
            condition=IfCondition(LaunchConfiguration('multiplot'))
        ),
        Node(
            package='ocs2_legged_robot_mpcnet',
            executable='legged_robot_mpcnet_dummy',
            name='legged_robot_mpcnet_dummy',
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile'),
                    'urdfFile': LaunchConfiguration('urdfFile'),
                    'referenceFile': LaunchConfiguration('referenceFile'),
                    'raisimFile': LaunchConfiguration('raisimFile'),
                    'resourcePath': LaunchConfiguration('resourcePath'),
                    'policyFile': LaunchConfiguration('policyFile'),
                    'useRaisim': LaunchConfiguration('useRaisim')
                }
            ]
        ),
        Node(
            package='ocs2_legged_robot_ros',
            executable='legged_robot_target',
            name='legged_robot_target',
            output='screen',
            prefix=prefix,
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile'),
                    'referenceFile': LaunchConfiguration('referenceFile'),
                    'urdfFile': LaunchConfiguration('urdfFile'),
                    'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
                }
            ]
        ),
        Node(
            package='ocs2_legged_robot_ros',
            executable='legged_robot_gait_command',
            name='legged_robot_gait_command',
            output='screen',
            prefix=prefix,
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile'),
                    'referenceFile': LaunchConfiguration('referenceFile'),
                    'urdfFile': LaunchConfiguration('urdfFile'),
                    'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
                }
            ]
        )
    ])