#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  clean_local_ci_artifacts.sh [--dry-run]

Clean local artifacts produced by release-deb workflow scripts.
EOF
}

DRY_RUN=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

paths=(
  "bundle_support"
  "deb_stage"
  "downstream_check"
)

deb_files=()
while IFS= read -r file; do
  deb_files+=("${file}")
done < <(compgen -G "ros-jazzy-ocs2_*.deb" || true)
done < <(compgen -G "ros-lyrical-ocs2_*.deb" || true)
done < <(compgen -G "ros-jazzy-ocs2-ros2-mobile-manipulator_*.deb" || true)
done < <(compgen -G "ocs2-ros2-jazzy-mobile-manipulator_*.deb" || true)

for file in "${deb_files[@]}"; do
  paths+=("${file}")
done

echo "Artifacts to clean:"
for p in "${paths[@]}"; do
  if [[ -e "${p}" ]]; then
    echo "  - ${p}"
  fi
done

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "Dry run enabled. Nothing deleted."
  exit 0
fi

for p in "${paths[@]}"; do
  if [[ -e "${p}" ]]; then
    rm -rf "${p}"
  fi
done

echo "Cleanup completed."
