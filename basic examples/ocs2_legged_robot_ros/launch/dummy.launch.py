import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    prefix = os.getenv('LAUNCH_PREFIX', '')
    return LaunchDescription([
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

        Node(
            package='ocs2_legged_robot_ros',
            executable='legged_robot_dummy',
            name='legged_robot_dummy',
            output='screen',
            prefix=prefix,
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
            ]
        )
    ])
