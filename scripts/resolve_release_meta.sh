#!/usr/bin/env bash
set -euo pipefail

DEFAULT_ROS_DISTRO="${DEFAULT_ROS_DISTRO:-jazzy}"
DEB_PACKAGE_NAME="${DEB_PACKAGE_NAME:-ocs2-ros2-jazzy-mobile-manipulator}"
DEB_FILE_PREFIX="${DEB_FILE_PREFIX:-${DEB_PACKAGE_NAME}}"

ros_distro="${INPUT_ROS_DISTRO:-${1:-}}"
if [[ -z "${ros_distro}" ]]; then
  ros_distro="${DEFAULT_ROS_DISTRO}"
fi

release_tag="${RELEASE_TAG_OVERRIDE:-${GITHUB_REF_NAME:-}}"
if [[ -z "${release_tag}" ]]; then
  release_tag="${GITHUB_EVENT_RELEASE_TAG_NAME:-}"
fi
if [[ -z "${release_tag}" ]]; then
  release_tag="local"
fi

deb_version="${INPUT_DEB_VERSION:-${2:-}}"
if [[ -z "${deb_version}" ]]; then
  deb_version="${release_tag#v}"
fi
if [[ -z "${deb_version}" ]]; then
  deb_version="0.0.0"
fi
deb_version="${deb_version#v}"
deb_version="${deb_version#V}"
deb_version="${deb_version//\//-}"
deb_file="${DEB_FILE_PREFIX}_${deb_version}_amd64.deb"

echo "ros_distro=${ros_distro}"
echo "release_tag=${release_tag}"
echo "deb_version=${deb_version}"
echo "deb_file=${deb_file}"
