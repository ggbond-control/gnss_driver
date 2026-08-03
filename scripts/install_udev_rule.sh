#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run this script with sudo."
  exit 1
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
install -m 0644 "${script_dir}/../udev/99-g60-gnss.rules" /etc/udev/rules.d/99-g60-gnss.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
echo "Installed 99-g60-gnss.rules. Reconnect the receiver if /dev/g60_gnss is not present."
