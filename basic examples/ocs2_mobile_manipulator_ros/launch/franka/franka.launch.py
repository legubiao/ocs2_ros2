import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    rviz = launch.actions.DeclareLaunchArgument(
        name='rviz',
        default_value='true'
    )

    debug = launch.actions.DeclareLaunchArgument(
        name='debug',
        default_value='false'
    )

    urdfFile = launch.actions.DeclareLaunchArgument(
        name='urdfFile',
        default_value=get_package_share_directory(
            'ocs2_robotic_assets') + '/resources/mobile_manipulator/franka/urdf/panda.urdf'
    )

    taskFile = launch.actions.DeclareLaunchArgument(
        name='taskFile',
        default_value=get_package_share_directory(
            'ocs2_mobile_manipulator') + '/config/franka/task.info'
    )

    libFolder = launch.actions.DeclareLaunchArgument(
        name='libFolder',
        default_value=get_package_share_directory(
            'ocs2_mobile_manipulator') + '/auto_generated/franka'
    )

    visualize_only = launch.actions.DeclareLaunchArgument(
        name='visualize_only',
        default_value='false',
        description='If true, only launch visualization without mobile manipulator'
    )

    mobile_manipulator = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'ocs2_mobile_manipulator_ros'), 'launch/include/mobile_manipulator.launch.py')
        ),
        launch_arguments={
            'rviz': launch.substitutions.LaunchConfiguration('rviz'),
            'debug': launch.substitutions.LaunchConfiguration('debug'),
            'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
            'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
            'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('visualize_only'))
    )

    visualize = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
        ),
        launch_arguments={
            'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
            'rviz': launch.substitutions.LaunchConfiguration('rviz'),
            'test': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('visualize_only'))
    )


    return LaunchDescription([
        rviz,
        debug,
        urdfFile,
        taskFile,
        libFolder,
        visualize_only,
        mobile_manipulator,
        visualize
    ])
