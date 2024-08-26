from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    multiplot_arg = DeclareLaunchArgument('multiplot', default_value='false')
    task_name_arg = DeclareLaunchArgument('task_name', default_value='mpc')
    policy_file_path_arg = DeclareLaunchArgument('policy_file_path', default_value=os.path.join(
        get_package_share_directory('ocs2_ballbot_mpcnet'), 'policy', 'ballbot.onnx'))

    rviz_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(
                        get_package_share_directory('ocs2_ballbot_ros'), 'launch', 'visualize.launch.py')]),
                launch_arguments={
                    'use_joint_state_publisher': 'false'
                }.items()
            ),
        ]
    )

    multiplot_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('multiplot')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ocs2_ballbot_ros'), 'launch', 'multiplot.launch.py')])
            )
        ]
    )

    ballbot_mpcnet_dummy_node = Node(
        package='ocs2_ballbot_mpcnet',
        executable='ballbot_mpcnet_dummy',
        name='ballbot_mpcnet_dummy',
        output='screen',
        arguments=[LaunchConfiguration('task_name'), LaunchConfiguration('policy_file_path')]
    )

    ballbot_target_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='ocs2_ballbot_ros',
        executable='ballbot_target',
        name='ballbot_target',
        output='screen',
        prefix='gnome-terminal --',
        arguments=[LaunchConfiguration('task_name')],
    )

    return LaunchDescription([
        rviz_arg,
        multiplot_arg,
        task_name_arg,
        policy_file_path_arg,
        rviz_group,
        multiplot_group,
        ballbot_mpcnet_dummy_node,
        ballbot_target_node
    ])