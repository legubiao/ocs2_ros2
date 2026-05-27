import os

import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def _robot_state_publisher_node(context, *args, **kwargs):
    # NOTE: description_name doubles as the URDF file path here for historical
    # reasons (the C++ MPC node also takes it as argv[1] -> getUrdfString()).
    # In ROS 2 Jazzy/Lyrical robot_state_publisher requires the URDF as a
    # `robot_description` parameter, so we parse the file here.
    urdf_file = context.launch_configurations['description_name']
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
            name='config_name',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='target_command',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='description_name',
            default_value='ocs2_anymal_description'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_quadruped_interface'), 'launch/visualization.launch.py')
            ),
            launch_arguments={
                'description_name': LaunchConfiguration('description_name'),
            }.items()
        ),
        OpaqueFunction(function=_robot_state_publisher_node),
        Node(
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_mpc_node',
            name='ocs2_anymal_mpc_mpc_node',
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        Node(
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_dummy_mrt_node',
            name='ocs2_anymal_mpc_dummy_mrt_node',
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
        Node(
            package='ocs2_anymal_commands',
            executable='motion_command_node',
            name='motion_command_node',
            prefix=prefix,
            arguments=['dummy'],
            output='screen'
        ),
    ])
