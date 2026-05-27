#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  build_release_deb.sh --ros-distro <distro> --deb-version <version> --release-tag <tag>
                       [--deb-package-name <name>] [--deb-file-prefix <prefix>]
                       [--deb-arch <arch>] [--install-prefix <prefix>]
                       [--required-packages "<pkg1 pkg2 ...>"] [--skip-deps] [--skip-colcon]

Build OCS2 packages (--packages-up-to roots) and create a bundled .deb package.
EOF
}

ROS_DISTRO=""
DEB_VERSION=""
RELEASE_TAG=""
DEB_PACKAGE_NAME=""
DEB_FILE_PREFIX=""
# Space-separated colcon --packages-up-to roots (all basic example * _ros packages).
DEFAULT_REQUIRED_PACKAGES="ocs2_ballbot_ros ocs2_cartpole_ros ocs2_double_integrator_ros ocs2_legged_robot_ros ocs2_mobile_manipulator_ros ocs2_quadrotor_ros"
REQUIRED_PACKAGES="${REQUIRED_OCS2_PACKAGES:-${DEFAULT_REQUIRED_PACKAGES}}"
DEB_ARCH="${DEB_ARCH:-}"
INSTALL_PREFIX=""
SKIP_DEPS=0
SKIP_COLCON=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --deb-version) DEB_VERSION="$2"; shift 2 ;;
    --release-tag) RELEASE_TAG="$2"; shift 2 ;;
    --deb-package-name) DEB_PACKAGE_NAME="$2"; shift 2 ;;
    --deb-file-prefix) DEB_FILE_PREFIX="$2"; shift 2 ;;
    --deb-arch) DEB_ARCH="$2"; shift 2 ;;
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

DEB_PACKAGE_NAME="${DEB_PACKAGE_NAME:-ros-${ROS_DISTRO}-ocs2}"
DEB_FILE_PREFIX="${DEB_FILE_PREFIX:-${DEB_PACKAGE_NAME}}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/opt/ros/${ROS_DISTRO}}"

if [[ -z "${REQUIRED_PACKAGES// }" ]]; then
  echo "Required package roots are empty. Pass --required-packages or set REQUIRED_OCS2_PACKAGES."
  exit 1
fi

DEB_VERSION="${DEB_VERSION#v}"
DEB_VERSION="${DEB_VERSION//\//-}"
if [[ -z "${DEB_VERSION}" ]]; then
  echo "Resolved deb version is empty after normalization."
  exit 1
fi

if [[ -z "${DEB_ARCH}" ]]; then
  DEB_ARCH="$(dpkg --print-architecture)"
fi
if [[ -z "${DEB_ARCH}" ]]; then
  echo "Unable to resolve Debian architecture."
  exit 1
fi

DEB_FILE="${DEB_FILE_PREFIX}_${DEB_VERSION}_${DEB_ARCH}.deb"
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

  COLCON_CMAKE_ARGS=(
    --cmake-args
    --no-warn-unused-cli
    -DBUILD_TESTING=OFF
    -DCMAKE_BUILD_TYPE=Release
    -DOCS2_USE_SYSTEM_BLASFEO=OFF
    -DOCS2_USE_SYSTEM_HPIPM=OFF
  )
  if command -v ccache >/dev/null 2>&1; then
    export CCACHE_BASEDIR="${CCACHE_BASEDIR:-${PWD}}"
    export CCACHE_NOHASHDIR="${CCACHE_NOHASHDIR:-true}"
    mkdir -p "${CCACHE_DIR:-${HOME}/.ccache}"
    ccache --zero-stats || true
    COLCON_CMAKE_ARGS+=(
      -DCMAKE_C_COMPILER_LAUNCHER=ccache
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    )
  fi

  # Resolve the full dependency closure, then --packages-select that set explicitly.
  # A hand-picked --packages-select list omits ocs2_sqp / hpipm_colcon / blasfeo_colcon
  # and fails ocs2_mobile_manipulator with missing package.sh (CI saw 12/15 success).
  # shellcheck disable=SC2206
  colcon_roots=(${REQUIRED_PACKAGES})
  mapfile -t colcon_packages < <(
    colcon list --names-only --packages-up-to "${colcon_roots[@]}"
  )
  if [[ ${#colcon_packages[@]} -eq 0 ]]; then
    echo "colcon list --packages-up-to returned no packages for: ${REQUIRED_PACKAGES}" >&2
    exit 1
  fi
  echo "Colcon closure (${#colcon_packages[@]} packages): ${colcon_packages[*]}"
  for required_pkg in blasfeo_colcon hpipm_colcon ocs2_sqp; do
    if [[ ! " ${colcon_packages[*]} " =~ [[:space:]]${required_pkg}[[:space:]] ]]; then
      echo "Missing ${required_pkg} in colcon closure; check package.xml depends." >&2
      exit 1
    fi
  done

  # No --symlink-install: release .deb must contain real files; symlinks to CI
  # workspace paths break on any other machine (including the consume_deb job).
  colcon build \
    --merge-install \
    --packages-select "${colcon_packages[@]}" \
    --install-base "${INSTALL_ROOT}" \
    "${COLCON_CMAKE_ARGS[@]}"

  if command -v ccache >/dev/null 2>&1; then
    ccache --show-stats || true
  fi

  for required_pkg in blasfeo_colcon hpipm_colcon ocs2_sqp; do
    pkg_sh="${INSTALL_ROOT}/share/${required_pkg}/package.sh"
    if [[ ! -f "${pkg_sh}" ]]; then
      echo "Expected install artifact missing: ${pkg_sh}" >&2
      exit 1
    fi
  done
  # shellcheck disable=SC2206
  basic_example_ros=(${DEFAULT_REQUIRED_PACKAGES})
  for required_pkg in "${basic_example_ros[@]}"; do
    pkg_sh="${INSTALL_ROOT}/share/${required_pkg}/package.sh"
    if [[ ! -f "${pkg_sh}" ]]; then
      echo "Expected basic example install artifact missing: ${pkg_sh}" >&2
      exit 1
    fi
  done
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
DEB_DEPENDS=(
  "libc6 (>= 2.35)"
  "ros-${ROS_DISTRO}-ros-base"
  "ros-${ROS_DISTRO}-geometry-msgs"
  "ros-${ROS_DISTRO}-interactive-markers"
  "ros-${ROS_DISTRO}-kdl-parser"
  "ros-${ROS_DISTRO}-pinocchio"
  "ros-${ROS_DISTRO}-rclcpp"
  "ros-${ROS_DISTRO}-rclcpp-lifecycle"
  "ros-${ROS_DISTRO}-robot-state-publisher"
  "ros-${ROS_DISTRO}-rosidl-default-runtime"
  "ros-${ROS_DISTRO}-std-msgs"
  "ros-${ROS_DISTRO}-tf2"
  "ros-${ROS_DISTRO}-tf2-ros"
  "ros-${ROS_DISTRO}-urdf"
  "ros-${ROS_DISTRO}-urdfdom"
  "ros-${ROS_DISTRO}-visualization-msgs"
  "ros-${ROS_DISTRO}-xacro"
  "xterm"
)
printf -v DEB_DEPENDS_LINE "%s, " "${DEB_DEPENDS[@]}"
DEB_DEPENDS_LINE="${DEB_DEPENDS_LINE%, }"

deb_provides=""
if [[ "${ROS_DISTRO}" == "jazzy" ]]; then
  deb_provides="Provides: ocs2-ros2-jazzy-mobile-manipulator, ros-jazzy-ocs2-ros2-mobile-manipulator"
fi

# Do not emit a blank line when deb_provides is empty: dpkg treats that as a second
# package stanza and fails with "several package info entries found".
{
  cat <<EOF
Package: ${DEB_PACKAGE_NAME}
Version: ${DEB_VERSION}
Section: libs
Priority: optional
Architecture: ${DEB_ARCH}
Maintainer: ocs2_ros2 CI <noreply@github.com>
Depends: ${DEB_DEPENDS_LINE}
EOF
  if [[ -n "${deb_provides}" ]]; then
    printf '%s\n' "${deb_provides}"
  fi
  cat <<EOF
Installed-Size: ${INSTALLED_SIZE_KB}
Description: Prebuilt OCS2 ROS2 bundle (core + all basic examples)
 Built from ${GITHUB_REPOSITORY:-local/ocs2_ros2} at tag/ref ${RELEASE_TAG}.
 Installed under ${INSTALL_PREFIX}.
EOF
} > "${DEBIAN_DIR}/control"

dpkg-deb --build "${STAGE_ROOT}" "${DEB_FILE}"
echo "Built deb: ${DEB_FILE}"

if [[ -n "${GITHUB_OUTPUT:-}" ]]; then
  {
    echo "deb_file=${DEB_FILE}"
    echo "deb_version=${DEB_VERSION}"
    echo "release_tag=${RELEASE_TAG}"
    echo "ros_distro=${ROS_DISTRO}"
    echo "deb_arch=${DEB_ARCH}"
  } >> "${GITHUB_OUTPUT}"
fi
