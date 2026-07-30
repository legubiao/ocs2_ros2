#!/usr/bin/env bash
set -euo pipefail

DEFAULT_ROS_DISTRO="${DEFAULT_ROS_DISTRO:-jazzy}"
DEB_ARCH="${DEB_ARCH:-}"
PRE_RELEASE_TAG="${PRE_RELEASE_TAG:-pre-release}"

ros_distro="${INPUT_ROS_DISTRO:-${1:-}}"
if [[ -z "${ros_distro}" ]]; then
  ros_distro="${DEFAULT_ROS_DISTRO}"
fi

DEB_PACKAGE_NAME="${DEB_PACKAGE_NAME:-ros-${ros_distro}-ocs2}"
DEB_FILE_PREFIX="${DEB_FILE_PREFIX:-${DEB_PACKAGE_NAME}}"

is_prerelease="false"
if [[ "${IS_PRERELEASE:-}" == "true" ]]; then
  is_prerelease="true"
fi

release_tag="${INPUT_RELEASE_TAG:-}"
if [[ -z "${release_tag}" ]]; then
  release_tag="${RELEASE_TAG_OVERRIDE:-${GITHUB_REF_NAME:-}}"
fi
if [[ -z "${release_tag}" ]]; then
  release_tag="${GITHUB_EVENT_RELEASE_TAG_NAME:-}"
fi

if [[ "${release_tag}" == "${PRE_RELEASE_TAG}" ]]; then
  is_prerelease="true"
fi

if [[ "${is_prerelease}" == "true" ]]; then
  release_tag="${PRE_RELEASE_TAG}"
elif [[ -z "${release_tag}" ]]; then
  echo "A release tag is required when not running from a tag ref." >&2
  exit 1
elif [[ "${release_tag}" != [vV]* ]]; then
  release_tag="v${release_tag}"
fi

deb_version="${INPUT_DEB_VERSION:-${2:-}}"
if [[ -z "${deb_version}" ]]; then
  if [[ "${is_prerelease}" == "true" ]]; then
    echo "INPUT_DEB_VERSION is required for pre-release builds." >&2
    exit 1
  fi
  deb_version="${release_tag#v}"
fi
if [[ -z "${deb_version}" ]]; then
  deb_version="0.0.0"
fi
deb_version="${deb_version#v}"
deb_version="${deb_version#V}"
deb_version="${deb_version//\//-}"
# Formal tags may use '-' as a prerelease separator; map to Debian '~'.
if [[ "${is_prerelease}" != "true" ]]; then
  deb_version="$(printf '%s' "${deb_version}" | sed 's/-/~/g')"
fi

deb_arch="${INPUT_DEB_ARCH:-${3:-}}"
if [[ -z "${deb_arch}" ]]; then
  deb_arch="${DEB_ARCH}"
fi
if [[ -z "${deb_arch}" ]]; then
  deb_arch="$(dpkg --print-architecture)"
fi
if [[ -z "${deb_arch}" ]]; then
  echo "Unable to resolve Debian architecture." >&2
  exit 1
fi

# GitHub Release rewrites "~" to "." in uploaded asset filenames; keep local
# and release asset names aligned with that convention.
# Escape '~' in the pattern: unescaped tilde is not matched by ${var//~/}.
deb_file_version="${deb_version//\~/.}"
deb_file="${DEB_FILE_PREFIX}_${deb_file_version}_${deb_arch}.deb"

echo "ros_distro=${ros_distro}"
echo "release_tag=${release_tag}"
echo "deb_version=${deb_version}"
echo "deb_file_version=${deb_file_version}"
echo "deb_arch=${deb_arch}"
echo "deb_file=${deb_file}"
echo "is_prerelease=${is_prerelease}"
