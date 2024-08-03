import os
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdfFile',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='taskFile',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator_ros') + "/rviz/mobile_manipulator_distance.rviz"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
            ),
            launch_arguments={
                'urdfFile': LaunchConfiguration('urdfFile'),
                'rvizconfig': LaunchConfiguration('rvizconfig')
            }.items()
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='mobile_manipulator_joint_state_publisher',
            parameters=[
                {
                    'rate': "100"
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_distance_visualization',
            name='mobile_manipulator_distance_visualization',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                }
            ]
        )
    ])
