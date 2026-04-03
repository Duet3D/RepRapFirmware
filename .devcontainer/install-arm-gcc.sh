#!/bin/sh

set -eu

arch="$(dpkg --print-architecture)"
case "${arch}" in
    amd64) arm_gnu_toolchain_host_arch="x86_64" ;;
    arm64) arm_gnu_toolchain_host_arch="aarch64" ;;
    *)
        echo "Unsupported architecture: ${arch}" >&2
        exit 1
        ;;
esac

arm_gnu_toolchain_archive="arm-gnu-toolchain-${ARM_GNU_TOOLCHAIN_VERSION}-${arm_gnu_toolchain_host_arch}-arm-none-eabi"
arm_gnu_toolchain_url="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_GNU_TOOLCHAIN_VERSION}/binrel/${arm_gnu_toolchain_archive}.tar.xz"

curl -L "${arm_gnu_toolchain_url}" -o "/tmp/${arm_gnu_toolchain_archive}.tar.xz"
tar -xJf "/tmp/${arm_gnu_toolchain_archive}.tar.xz" -C /opt
ln -s "/opt/${arm_gnu_toolchain_archive}" "${ARM_GNU_TOOLCHAIN_DIR}"
mkdir -p /workspaces
ln -s "/opt/${arm_gnu_toolchain_archive}" "/workspaces/${arm_gnu_toolchain_archive}"
rm -f "/tmp/${arm_gnu_toolchain_archive}.tar.xz"