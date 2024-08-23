import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_double_integrator_ros') + "/rviz/double_integrator.rviz"
    urdf_dir = get_package_share_directory("ocs2_robotic_assets")
    urdf_model_path = os.path.join(urdf_dir, "resources/double_integrator/urdf", "double_integrator.urdf")

    # Read the URDF file
    with open(urdf_model_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='use_joint_state_publisher',
            default_value='true'
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{'robot_description': urdf_content}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            arguments=[urdf_model_path],
            condition=IfCondition(use_joint_state_publisher),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='double_integrator',
            output='screen',
            arguments=["-d", rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])
