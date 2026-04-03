#!/bin/bash
# Build RepRapFirmware headless using Eclipse CDT
# Assumes 'eclipse' is on PATH and ArmGccPath is set or passed as first argument

ECLIPSE_ARGS="--launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild"

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
RRF_ROOT="$SCRIPT_DIR/.."
LIBRARIES_ROOT="$RRF_ROOT/libraries"
WORKSPACE="$RRF_ROOT/headless-workspace"

ARM_GNU_TOOLCHAIN_VERSION="${ARM_GNU_TOOLCHAIN_VERSION:-15.2.rel1}"

case "$(uname -m)" in
    aarch64|arm64)
        ARM_GNU_TOOLCHAIN_HOST_ARCH="aarch64"
        ;;
    x86_64|amd64)
        ARM_GNU_TOOLCHAIN_HOST_ARCH="x86_64"
        ;;
    *)
        echo "Unsupported host architecture: $(uname -m)" >&2
        exit 1
        ;;
esac

DEFAULT_GCC_PATH="$RRF_ROOT/../arm-gnu-toolchain-${ARM_GNU_TOOLCHAIN_VERSION}-${ARM_GNU_TOOLCHAIN_HOST_ARCH}-arm-none-eabi/bin"
ARM_GCC_PATH="${1:-${ArmGccPath:-$DEFAULT_GCC_PATH}}"

set -e

trap 'echo "=== Cleaning up workspace ==="; rm -rf "$WORKSPACE"' EXIT

echo "=== Initializing library submodules ==="
git -C "$RRF_ROOT" submodule update --init --recursive -- \
    libraries/CANlib \
    libraries/CoreN2G \
    libraries/FreeRTOS \
    libraries/RRFLibraries \
    libraries/WiFiSocketServerRTOS \
    libraries/LibTinyusb \
    libraries/LibMbedTls

echo "=== Importing projects into workspace ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" \
    -import "$LIBRARIES_ROOT/CANlib" \
    -import "$LIBRARIES_ROOT/CoreN2G" \
    -import "$LIBRARIES_ROOT/FreeRTOS" \
    -import "$LIBRARIES_ROOT/LibMbedTls" \
    -import "$LIBRARIES_ROOT/LibTinyusb" \
    -import "$LIBRARIES_ROOT/RRFLibraries" \
    -import "$LIBRARIES_ROOT/WiFiSocketServerRTOS" \
    -import "$RRF_ROOT"

echo "=== Building RepRapFirmware/Duet3Mini5plus ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "RepRapFirmware/Duet3Mini5plus"

echo "=== Building RepRapFirmware/Duet3_MB6HC ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "RepRapFirmware/Duet3_MB6HC"

echo "=== Building RepRapFirmware/Duet3_MB6XD ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "RepRapFirmware/Duet3_MB6XD"

echo "=== Build complete ==="
