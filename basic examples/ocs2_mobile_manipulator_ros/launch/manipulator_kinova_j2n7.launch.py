import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def _detect_pinocchio_version():
    """Best-effort detection of the installed pinocchio version (string or None).

    Tries ``pkg-config --modversion pinocchio`` first, then scans common
    ``pinocchio/config.hpp`` locations and extracts ``PINOCCHIO_VERSION``.
    """
    import subprocess
    try:
        out = subprocess.check_output(
            ['pkg-config', '--modversion', 'pinocchio'],
            stderr=subprocess.DEVNULL, text=True, timeout=2,
        ).strip()
        if out:
            return out
    except (OSError, subprocess.SubprocessError):
        pass
    import re
    pattern = re.compile(r'#\s*define\s+PINOCCHIO_VERSION\s+"([^"]+)"')
    for hdr in ('/opt/ros/jazzy/include/pinocchio/config.hpp',
                '/opt/ros/kilted/include/pinocchio/config.hpp',
                '/usr/include/pinocchio/config.hpp'):
        try:
            with open(hdr, 'r') as fh:
                for line in fh:
                    m = pattern.search(line)
                    if m:
                        return m.group(1)
        except OSError:
            continue
    return None


def _ocs2_codegen_dir(pkg, sub=''):
    """Resolve CppAD code-gen cache directory for *pkg*.

    Default: ``/tmp/ocs2_<USER>_<UID>/<pkg>/auto_generated[/pinocchio-<ver>]/<sub>``
    Override via env var ``OCS2_CODEGEN_DIR``.

    Living under /tmp means a fresh boot guarantees the cache is rebuilt, so
    system upgrades (e.g. pinocchio) cannot silently leave stale ABI .so.
    Additionally, when pinocchio is detectable on this host, the cache path is
    suffixed with the pinocchio version — apt-upgrading pinocchio thus moves
    the cache to a fresh subdirectory automatically, without a reboot.
    """
    base = os.environ.get('OCS2_CODEGEN_DIR') or \
        f"/tmp/ocs2_{os.environ.get('USER', 'anon')}_{os.getuid()}"
    parts = [base, pkg, 'auto_generated']
    version = _detect_pinocchio_version()
    if version:
        parts.append(f'pinocchio-{version}')
    if sub:
        parts.extend(p for p in sub.split('/') if p)
    return os.path.join(*parts)


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'ocs2_robotic_assets') + '/resources/mobile_manipulator/kinova/urdf/j2n7s300.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator') + '/config/kinova/task_j2n7.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=_ocs2_codegen_dir('ocs2_mobile_manipulator', 'kinova/j2n7')
        ),
        launch.actions.IncludeLaunchDescription(
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
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
