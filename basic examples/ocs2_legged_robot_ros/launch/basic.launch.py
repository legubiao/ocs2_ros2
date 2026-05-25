import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def _robot_state_publisher_node(context, *args, **kwargs):
    # robot_state_publisher in ROS 2 Jazzy/Lyrical no longer accepts a URDF
    # file path as a positional argument. Parse the file here so we can pass
    # the URDF/XML string as the `robot_description` parameter. Using xacro
    # makes future xacro-ization a no-op.
    urdf_file = context.launch_configurations['urdfFile']
    robot_description = xacro.process_file(urdf_file).toxml()
    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'publish_frequency': 100.0,
            'use_tf_static': True,
            'robot_description': robot_description,
        }],
    )]


def generate_launch_description():
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
            name='urdfFile',
            default_value=get_package_share_directory(
                'ocs2_robotic_assets') + '/resources/anymal_c/urdf/anymal.urdf'
        ),

        OpaqueFunction(function=_robot_state_publisher_node),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])
