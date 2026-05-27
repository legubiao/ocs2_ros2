"""Shared helpers for OCS2 ROS 2 launch files."""
import os
import re
import subprocess


def detect_pinocchio_version():
    """Best-effort pinocchio version string, or None."""
    try:
        out = subprocess.check_output(
            ['pkg-config', '--modversion', 'pinocchio'],
            stderr=subprocess.DEVNULL, text=True, timeout=2,
        ).strip()
        if out:
            return out
    except (OSError, subprocess.SubprocessError):
        pass
    pattern = re.compile(r'#\s*define\s+PINOCCHIO_VERSION\s+"([^"]+)"')
    for hdr in (
        '/opt/ros/jazzy/include/pinocchio/config.hpp',
        '/opt/ros/lyrical/include/pinocchio/config.hpp',
        '/opt/ros/kilted/include/pinocchio/config.hpp',
        '/usr/include/pinocchio/config.hpp',
    ):
        try:
            with open(hdr, 'r', encoding='utf-8') as fh:
                for line in fh:
                    match = pattern.search(line)
                    if match:
                        return match.group(1)
        except OSError:
            continue
    return None


def ocs2_codegen_dir(package, sub=''):
    """CppAD code-gen cache directory (matches C++ getCodegenPath())."""
    base = os.environ.get('OCS2_CODEGEN_DIR') or \
        f"/tmp/ocs2_{os.environ.get('USER', 'anon')}_{os.getuid()}"
    parts = [base, package, 'auto_generated']
    version = detect_pinocchio_version()
    if version:
        parts.append(f'pinocchio-{version}')
    if sub:
        parts.extend(p for p in sub.split('/') if p)
    return os.path.join(*parts)


def ocs2_task_file(package, config_name):
    """Absolute path to config/<config_name>/task.info in an OCS2 package."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory(package),
        'config',
        config_name,
        'task.info',
    )


def ocs2_installed_autogen_dir(package):
    """Share-tree auto_generated directory (e.g. ocs2_double_integrator)."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), 'auto_generated')
