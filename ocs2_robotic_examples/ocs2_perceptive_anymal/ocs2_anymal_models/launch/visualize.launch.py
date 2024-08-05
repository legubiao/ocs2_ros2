from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_urdf_file(robot_name):
    if robot_name == 'anymal_c':
        urdf_file = get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
    elif robot_name == 'camel':
        urdf_file = get_package_share_directory('ocs2_anymal_models') + "/urdf/anymal_camel_rsl.urdf"
    else:
        raise ValueError("Invalid robot_name: " + robot_name)
    return urdf_file


def launch_setup(context, *args, **kwargs):
    rviz_config_file = get_package_share_directory('ocs2_anymal_models') + "/config/visualize_urdf.rviz"

    robot_name = LaunchConfiguration('robot_name').perform(context)
    print("Visualizing robot: " + robot_name)
    urdf_file = get_urdf_file(robot_name)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'publish_frequency': 100.0,
                    'use_tf_static': True,
                },
            ],
            arguments=[urdf_file],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_gui': True,
                    'rate': 100.0
                },
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name',
            default_value='anymal_c',
            description='Name of the robot to visualize'
        ),
        OpaqueFunction(function=launch_setup)
    ])
