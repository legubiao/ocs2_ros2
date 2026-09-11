#!/usr/bin/env bash
# Rewrite Canonical Ubuntu apt URIs to Azure-first mirror lists.
# ros:* containers do not inherit GitHub-hosted runner Azure mirrors, so
# archive.ubuntu.com / ports.ubuntu.com can stall for tens of minutes.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive

cat >/etc/apt/apt.conf.d/99-ci-acquire <<'EOF'
Acquire::Retries "1";
Acquire::http::Timeout "15";
Acquire::https::Timeout "15";
Acquire::Languages "none";
Acquire::IndexTargets::deb::DEP-11::DefaultEnabled "false";
EOF

mirrors_file=/etc/apt/ci-ubuntu-mirrors.txt
if [[ "$(dpkg --print-architecture)" == "arm64" ]]; then
  printf '%s\n' \
    $'http://azure.ports.ubuntu.com/ubuntu-ports/\tpriority:1' \
    $'http://ports.ubuntu.com/ubuntu-ports/\tpriority:2' \
    > "$mirrors_file"
else
  printf '%s\n' \
    $'http://azure.archive.ubuntu.com/ubuntu/\tpriority:1' \
    $'http://archive.ubuntu.com/ubuntu/\tpriority:2' \
    $'http://security.ubuntu.com/ubuntu/\tpriority:3' \
    > "$mirrors_file"
fi

rewrite_ubuntu_mirrors() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  sed -E -i \
    -e 's#https?://(archive|security)\.ubuntu\.com/ubuntu/?#mirror+file:/etc/apt/ci-ubuntu-mirrors.txt#g' \
    -e 's#https?://ports\.ubuntu\.com/ubuntu-ports/?#mirror+file:/etc/apt/ci-ubuntu-mirrors.txt#g' \
    "$f"
}
rewrite_ubuntu_mirrors /etc/apt/sources.list
shopt -s nullglob
for f in /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
  rewrite_ubuntu_mirrors "$f"
done

echo "Ubuntu apt mirrors ($(dpkg --print-architecture)):"
cat "$mirrors_file"
grep -RInE 'URIs:|^deb ' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true
