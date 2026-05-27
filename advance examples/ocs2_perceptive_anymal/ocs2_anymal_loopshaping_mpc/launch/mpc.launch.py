import os

import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _robot_state_publisher_node(context, *args, **kwargs):
    urdf_file = context.launch_configurations['urdf_model_path']
    robot_description = xacro.process_file(urdf_file).toxml()
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
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


    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name'
        ),
        DeclareLaunchArgument(
            name='config_name'
        ),
        DeclareLaunchArgument(  
            name='rviz',  
            default_value='true',  
        ) ,
        DeclareLaunchArgument(
            name='description_name',
            default_value='ocs2_anymal_description'
        ),
        DeclareLaunchArgument(
            name='target_command',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='urdf_model_path',
            default_value=get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
        ),
        OpaqueFunction(function=_robot_state_publisher_node),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('ocs2_quadruped_interface') + "/launch/visualization.launch.py"
            ),
            launch_arguments={
                'description_name': LaunchConfiguration('description_name'),
            }.items()
        ),
        Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_mpc_node',
            name='ocs2_anymal_loopshaping_mpc_mpc_node',
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_dummy_mrt_node',
            name='ocs2_anymal_loopshaping_mpc_dummy_mrt_node',
            prefix=prefix,
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        Node(
            package='ocs2_anymal_commands',
            executable='gait_command_node',
            name='gait_command_node',
            prefix=prefix,
            output='screen'
        ),
        Node(
            package='ocs2_anymal_commands',
            executable='target_command_node',
            name='target_command_node',
            prefix=prefix,
            arguments=[LaunchConfiguration('target_command')],
            output='screen'
        ),
    ])
