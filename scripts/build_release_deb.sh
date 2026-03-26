#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  build_release_deb.sh --ros-distro <distro> --deb-version <version> --release-tag <tag>
                       [--deb-package-name <name>] [--install-prefix <prefix>]
                       [--required-packages "<pkg1 pkg2 ...>"] [--skip-deps] [--skip-colcon]

Build selected OCS2 packages and create a bundled .deb package.
EOF
}

ROS_DISTRO=""
DEB_VERSION=""
RELEASE_TAG=""
DEB_PACKAGE_NAME="${DEB_PACKAGE_NAME:-ocs2-ros2-jazzy-mobile-manipulator}"
DEB_FILE_PREFIX="${DEB_FILE_PREFIX:-ocs2-ros2-jazzy-mobile-manipulator}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/opt/ros/jazzy}"
REQUIRED_PACKAGES="${REQUIRED_OCS2_PACKAGES:-}"
SKIP_DEPS=0
SKIP_COLCON=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --deb-version) DEB_VERSION="$2"; shift 2 ;;
    --release-tag) RELEASE_TAG="$2"; shift 2 ;;
    --deb-package-name) DEB_PACKAGE_NAME="$2"; shift 2 ;;
    --deb-file-prefix) DEB_FILE_PREFIX="$2"; shift 2 ;;
    --install-prefix) INSTALL_PREFIX="$2"; shift 2 ;;
    --required-packages) REQUIRED_PACKAGES="$2"; shift 2 ;;
    --skip-deps) SKIP_DEPS=1; shift ;;
    --skip-colcon) SKIP_COLCON=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

if [[ -z "${ROS_DISTRO}" || -z "${DEB_VERSION}" || -z "${RELEASE_TAG}" ]]; then
  echo "Missing required arguments."
  usage
  exit 1
fi

if [[ -z "${REQUIRED_PACKAGES// }" ]]; then
  echo "REQUIRED_OCS2_PACKAGES is empty. Pass --required-packages."
  exit 1
fi

DEB_VERSION="${DEB_VERSION#v}"
DEB_VERSION="${DEB_VERSION//\//-}"
if [[ -z "${DEB_VERSION}" ]]; then
  echo "Resolved deb version is empty after normalization."
  exit 1
fi

DEB_FILE="${DEB_FILE_PREFIX}_${DEB_VERSION}_amd64.deb"
STAGE_ROOT="${PWD}/deb_stage"
INSTALL_ROOT="${STAGE_ROOT}${INSTALL_PREFIX}"
DEBIAN_DIR="${STAGE_ROOT}/DEBIAN"

strip_prefix_common_files() {
  local prefix_root="$1"
  rm -f \
    "${prefix_root}/setup.bash" \
    "${prefix_root}/setup.sh" \
    "${prefix_root}/setup.zsh" \
    "${prefix_root}/setup.ps1" \
    "${prefix_root}/local_setup.bash" \
    "${prefix_root}/local_setup.sh" \
    "${prefix_root}/local_setup.zsh" \
    "${prefix_root}/local_setup.ps1" \
    "${prefix_root}/_local_setup_util.py" \
    "${prefix_root}/_local_setup_util_sh.py" \
    "${prefix_root}/_local_setup_util_ps1.py" \
    "${prefix_root}/.colcon_install_layout" \
    "${prefix_root}/COLCON_IGNORE"
}

rewrite_prefix_paths() {
  local source_prefix="$1"
  local runtime_prefix="$2"
  python3 - "$source_prefix" "$runtime_prefix" <<'PY'
import os
import sys

source_prefix = sys.argv[1]
runtime_prefix = sys.argv[2]

for dirpath, _, filenames in os.walk(source_prefix):
    for filename in filenames:
        path = os.path.join(dirpath, filename)
        try:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception:
            continue
        if source_prefix not in content:
            continue
        with open(path, "w", encoding="utf-8") as f:
            f.write(content.replace(source_prefix, runtime_prefix))
PY
}

# ament_cmake runs /usr/bin/python3 during configure; that process must see ROS
# site-packages. Some environments do not propagate PYTHONPATH from the shell
# into CMake's execute_process children reliably, so set it explicitly after sourcing ROS.
ensure_ros_pythonpath() {
  local ros_distro="$1"
  local pyver site_pkgs
  pyver="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  site_pkgs="/opt/ros/${ros_distro}/lib/python${pyver}/site-packages"
  if [[ -d "${site_pkgs}" ]]; then
    export PYTHONPATH="${site_pkgs}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
}

if [[ "${SKIP_DEPS}" -eq 0 ]]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  ensure_ros_pythonpath "${ROS_DISTRO}"
  rosdep install --from-paths . --ignore-src -r -y
fi

if [[ "${SKIP_COLCON}" -eq 0 ]]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  ensure_ros_pythonpath "${ROS_DISTRO}"
  rm -rf "${STAGE_ROOT}"
  mkdir -p "${INSTALL_ROOT}" "${DEBIAN_DIR}"
  # No --symlink-install: release .deb must contain real files; symlinks to CI
  # workspace paths break on any other machine (including the consume_deb job).
  colcon build \
    --merge-install \
    --packages-select ${REQUIRED_PACKAGES} \
    --install-base "${INSTALL_ROOT}"
fi

echo "[debug] Checking installed config files before packaging..."
find "${INSTALL_ROOT}" \( \
  -name 'ocs2_mobile_manipulatorConfig.cmake' -o \
  -name 'ocs2_mobile_manipulator-config.cmake' -o \
  -name 'ocs2_mobile_manipulator_rosConfig.cmake' -o \
  -name 'ocs2_mobile_manipulator_ros-config.cmake' -o \
  -name 'ocs2_ros_interfacesConfig.cmake' -o \
  -name 'ocs2_ros_interfaces-config.cmake' \
\) -print

rewrite_prefix_paths "${INSTALL_ROOT}" "${INSTALL_PREFIX}"
strip_prefix_common_files "${INSTALL_ROOT}"

INSTALLED_SIZE_KB="$(du -sk "${STAGE_ROOT}" | cut -f1)"
cat > "${DEBIAN_DIR}/control" <<EOF
Package: ${DEB_PACKAGE_NAME}
Version: ${DEB_VERSION}
Section: libs
Priority: optional
Architecture: amd64
Maintainer: ocs2_ros2 CI <noreply@github.com>
Depends: libc6 (>= 2.35), ros-${ROS_DISTRO}-ros-base
Description: Prebuilt OCS2 ROS2 bundle for selected mobile manipulator dependency chain
 Built from ${GITHUB_REPOSITORY:-local/ocs2_ros2} at tag/ref ${RELEASE_TAG}.
 Installed under ${INSTALL_PREFIX}.
Installed-Size: ${INSTALLED_SIZE_KB}
EOF

dpkg-deb --build "${STAGE_ROOT}" "${DEB_FILE}"
echo "Built deb: ${DEB_FILE}"

if [[ -n "${GITHUB_OUTPUT:-}" ]]; then
  {
    echo "deb_file=${DEB_FILE}"
    echo "deb_version=${DEB_VERSION}"
    echo "release_tag=${RELEASE_TAG}"
    echo "ros_distro=${ROS_DISTRO}"
  } >> "${GITHUB_OUTPUT}"
fi
