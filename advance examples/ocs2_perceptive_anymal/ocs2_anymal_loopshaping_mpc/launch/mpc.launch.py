from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def is_wsl():
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False

def generate_launch_description():
    
    prefix = "gnome-terminal --"
    if is_wsl():
        prefix = "xterm -e"
        print("Current system is WSL, use xterm as terminal")
    else:
        print("Current system is not WSL, use gnome-terminal as terminal")
        
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name'
        ),
        DeclareLaunchArgument(
            name='config_name'
        ),
        DeclareLaunchArgument(  
            name='rviz',  
            default_value='true',  
        ) ,
        DeclareLaunchArgument(
            name='description_name',
            default_value='ocs2_anymal_description'
        ),
        DeclareLaunchArgument(
            name='target_command',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='urdf_model_path',
            default_value=get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            arguments=[LaunchConfiguration("urdf_model_path")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('ocs2_quadruped_interface') + "/launch/visualization.launch.py"
            ),
            launch_arguments={
                'description_name': LaunchConfiguration('description_name'),
            }.items()
        ),
        Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_mpc_node',
            name='ocs2_anymal_loopshaping_mpc_mpc_node',
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_dummy_mrt_node',
            name='ocs2_anymal_loopshaping_mpc_dummy_mrt_node',
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
    ])
