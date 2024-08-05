import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            arguments=[LaunchConfiguration("description_name")],
        ),
        Node(
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_mpc_node',
            name='ocs2_anymal_mpc_mpc_node',
            prefix="",
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        Node(
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_dummy_mrt_node',
            name='ocs2_anymal_mpc_dummy_mrt_node',
            prefix="gnome-terminal --",
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        Node(
            package='ocs2_anymal_commands',
            executable='gait_command_node',
            name='gait_command_node',
            prefix="gnome-terminal --",
            output='screen'
        ),
        Node(
            package='ocs2_anymal_commands',
            executable='target_command_node',
            name='target_command_node',
            prefix="gnome-terminal --",
            arguments=[LaunchConfiguration('target_command')],
            output='screen'
        ),
        Node(
            package='ocs2_anymal_commands',
            executable='motion_command_node',
            name='motion_command_node',
            prefix="gnome-terminal --",
            arguments=['dummy'],
            output='screen'
        ),
    ])
