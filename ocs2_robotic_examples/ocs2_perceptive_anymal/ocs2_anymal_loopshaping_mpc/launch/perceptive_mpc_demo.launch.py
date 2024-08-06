from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_anymal_loopshaping_mpc') + "/config/rviz/demo_config.rviz"
    urdf_model_path = get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name',
            default_value='anymal_c'
        ),
        DeclareLaunchArgument(
            name='config_name',
            default_value='c_series'
        ),
        DeclareLaunchArgument(
            name='terrain_name',
            default_value='step.png'
        ),
        DeclareLaunchArgument(
            name='terrain_scale',
            default_value='0.35'
        ),
        DeclareLaunchArgument(
            name='forward_distance',
            default_value='3.0'
        ),
        DeclareLaunchArgument(
            name='perception_parameter_file',
            default_value=get_package_share_directory(
                'convex_plane_decomposition_ros') + '/config/parameters.yaml'
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_model_path],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            name='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            output='screen',
            parameters=[
                {
                    'config_name': LaunchConfiguration('config_name'),
                    'forward_velocity': 0.5,
                    'forward_distance': LaunchConfiguration('forward_distance'),
                    'terrain_name': LaunchConfiguration('terrain_name'),
                    'ocs2_anymal_description': urdf_model_path,
                    'terrain_scale': LaunchConfiguration('terrain_scale'),
                    'adaptReferenceToTerrain': True
                },
                LaunchConfiguration('perception_parameter_file')
            ]
        ),
    ])
