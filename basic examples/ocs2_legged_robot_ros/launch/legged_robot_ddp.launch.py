import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,  SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def is_wsl():
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False

def generate_launch_description():

    if shutil.which("terminator"):
        prefix = "terminator --new-tab -x"
        print("Terminator is installed, use terminator as terminal")
    elif is_wsl():
        prefix = "xterm -e"
        print("Current system is WSL, use xterm as terminal")
    else:
        prefix = "gnome-terminal --"
        print("Current system is not WSL, use gnome-terminal as terminal")


    return LaunchDescription([
        SetEnvironmentVariable('LAUNCH_PREFIX', prefix),
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
        DeclareLaunchArgument(
            name='gaitCommandFile',
            default_value=get_package_share_directory(
                'ocs2_legged_robot') + '/config/command/gait.info'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/basic.launch.py']),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'description_name': LaunchConfiguration('description_name'),
                'multiplot': LaunchConfiguration('multiplot'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/mpc_ddp.launch.py']),
            launch_arguments={
                'referenceFile': LaunchConfiguration('referenceFile'),
                'taskFile': LaunchConfiguration('taskFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/dummy.launch.py']),
            launch_arguments={
                'referenceFile': LaunchConfiguration('referenceFile'),
                'taskFile': LaunchConfiguration('taskFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/gait_command.launch.py']),
            launch_arguments={
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robot_target.launch.py']),
            launch_arguments={
                'referenceFile': LaunchConfiguration('referenceFile'),
            }.items(),
        ),
    ])
