#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  consume_release_deb.sh --deb-file <file> --ros-distro <distro>
                         [--install-prefix <prefix>] [--release-tag <tag>] [--repo <owner/repo>]
                         [--download-from-release] [--skip-install]

Install the bundled .deb, overlay the merged install on top of ROS (same as
"source /opt/ros/<distro>/setup.bash" then "source <prefix>/setup.bash"), then
run a tiny CMake project that find_package()'s key OCS2 packages — proves the
.deb is usable for downstream builds.
EOF
}

DEB_FILE=""
ROS_DISTRO=""
INSTALL_PREFIX=""
RELEASE_TAG=""
REPO="${GITHUB_REPOSITORY:-}"
DOWNLOAD_FROM_RELEASE=0
SKIP_INSTALL=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --deb-file) DEB_FILE="$2"; shift 2 ;;
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --install-prefix) INSTALL_PREFIX="$2"; shift 2 ;;
    --release-tag) RELEASE_TAG="$2"; shift 2 ;;
    --repo) REPO="$2"; shift 2 ;;
    --download-from-release) DOWNLOAD_FROM_RELEASE=1; shift ;;
    --skip-install) SKIP_INSTALL=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

if [[ -z "${DEB_FILE}" || -z "${ROS_DISTRO}" ]]; then
  echo "Missing required arguments."
  usage
  exit 1
fi

INSTALL_PREFIX="${INSTALL_PREFIX:-/opt/ros/${ROS_DISTRO}}"

if [[ "${DOWNLOAD_FROM_RELEASE}" -eq 1 ]]; then
  if [[ -z "${RELEASE_TAG}" || -z "${REPO}" ]]; then
    echo "--download-from-release requires --release-tag and --repo."
    exit 1
  fi
  gh release download "${RELEASE_TAG}" --repo "${REPO}" --pattern "${DEB_FILE}"
fi

if [[ "${SKIP_INSTALL}" -eq 0 ]]; then
  # Use apt (not bare dpkg -i) so Depends from the .deb control file are pulled
  # from the ROS apt repo (pinocchio, interactive-markers, xacro, xterm, ...).
  if [[ "$(id -u)" -eq 0 ]]; then
    apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "./${DEB_FILE}"
  else
    sudo apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "./${DEB_FILE}"
  fi
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# Colcon merge-install ships setup.bash; use it (do not rely on a hand-written
# setup.sh that might miss hooks). Fallbacks keep old .deb layouts working.
if [[ -f "${INSTALL_PREFIX}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${INSTALL_PREFIX}/setup.bash"
elif [[ -f "${INSTALL_PREFIX}/local_setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${INSTALL_PREFIX}/local_setup.bash"
elif [[ -f "${INSTALL_PREFIX}/ocs2_ros2_bundle_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "${INSTALL_PREFIX}/ocs2_ros2_bundle_env.sh"
else
  export CMAKE_PREFIX_PATH="${INSTALL_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export AMENT_PREFIX_PATH="${INSTALL_PREFIX}:${AMENT_PREFIX_PATH:-}"
fi
set -u

echo "[debug] INSTALL_PREFIX=${INSTALL_PREFIX}"
echo "[debug] CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-<empty>}"
echo "[debug] AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-<empty>}"

echo "[debug] Looking for package config files..."
find "${INSTALL_PREFIX}" \( \
  -name 'ocs2_mobile_manipulatorConfig.cmake' -o \
  -name 'ocs2_mobile_manipulator-config.cmake' -o \
  -name 'ocs2_mobile_manipulator_rosConfig.cmake' -o \
  -name 'ocs2_mobile_manipulator_ros-config.cmake' -o \
  -name 'ocs2_ros_interfacesConfig.cmake' -o \
  -name 'ocs2_ros_interfaces-config.cmake' \
\) -print

rm -rf downstream_check
mkdir -p downstream_check
cat > downstream_check/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(ocs2_downstream_check CXX)

# Suppress legacy FindBoost deprecation warning from ocs2_core transitive deps.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(ocs2_mobile_manipulator REQUIRED)
find_package(ocs2_mobile_manipulator_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)

add_executable(dummy main.cpp)
# ocs2_mobile_manipulator_ros installs nodes only (no ament_export_targets); linking
# its :: imported target fails. Link the exported libraries to validate .so wiring.
target_link_libraries(dummy PRIVATE
  ocs2_mobile_manipulator::ocs2_mobile_manipulator
  ocs2_ros_interfaces::ocs2_ros_interfaces
)
EOF

cat > downstream_check/main.cpp <<'EOF'
int main() { return 0; }
EOF

cmake -S downstream_check -B downstream_check/build
cmake --build downstream_check/build -j2
echo "Downstream CMake verification passed."
