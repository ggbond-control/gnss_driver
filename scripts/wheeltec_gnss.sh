#!/bin/sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
  echo "Run this script with sudo: sudo sh wheeltec_gnss.sh"
  exit 1
fi

# Resolve symlink if invoked through wheeltec_gnss.sh link
target="$0"
while [ -h "$target" ]; do
  dir="$(cd -P -- "$(dirname -- "$target")" && pwd)"
  target="$(readlink -- "$target")"
  case "$target" in
    /*) ;;
    *) target="$dir/$target" ;;
  esac
done
script_dir="$(cd -P -- "$(dirname -- "$target")" && pwd)"

if [ -f "${script_dir}/udev/99-wheeltec-gnss.rules" ]; then
  rules_file="${script_dir}/udev/99-wheeltec-gnss.rules"
elif [ -f "${script_dir}/../udev/99-wheeltec-gnss.rules" ]; then
  rules_file="${script_dir}/../udev/99-wheeltec-gnss.rules"
else
  echo "Error: rules file not found (searched ${script_dir}/udev and ${script_dir}/../udev)"
  exit 1
fi

install -m 0644 "$rules_file" /etc/udev/rules.d/99-wheeltec-gnss.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
echo "Installed 99-wheeltec-gnss.rules. /dev/wheeltec_gnss symlinks are now configured."
