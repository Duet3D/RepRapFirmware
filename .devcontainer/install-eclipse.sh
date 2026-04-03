#!/bin/sh

set -eu

arch="$(dpkg --print-architecture)"
case "${arch}" in
    amd64) eclipse_host_arch="x86_64" ;;
    arm64) eclipse_host_arch="aarch64" ;;
    *)
        echo "Unsupported architecture: ${arch}" >&2
        exit 1
        ;;
esac

eclipse_archive="${ECLIPSE_PACKAGE}-linux-gtk-${eclipse_host_arch}.tar.gz"
eclipse_url="https://download.eclipse.org/technology/epp/downloads/release/${ECLIPSE_RELEASE}/R/${eclipse_archive}"

curl -L "${eclipse_url}" -o "/tmp/${eclipse_archive}"
tar -xzf "/tmp/${eclipse_archive}" -C /opt
ln -sf "${ECLIPSE_DIR}/eclipse" /usr/local/bin/eclipse
rm -f "/tmp/${eclipse_archive}"