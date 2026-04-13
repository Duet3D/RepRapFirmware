#!/bin/bash
# Build RepRapFirmware headless using Eclipse CDT
# Assumes 'eclipse' is on PATH and ArmGccPath is set or passed as first argument

ECLIPSE_ARGS="--launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild"

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
RRF_ROOT="$SCRIPT_DIR/.."
PROJECTS_ROOT="$SCRIPT_DIR/../.."
WORKSPACE="$PROJECTS_ROOT/headless-workspace"

DEFAULT_GCC_PATH="$PROJECTS_ROOT/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin"
ARM_GCC_PATH="${1:-${ArmGccPath:-$DEFAULT_GCC_PATH}}"

set -e

trap 'echo "=== Cleaning up workspace ==="; rm -rf "$WORKSPACE"' EXIT

echo "=== Importing projects into workspace ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" \
    -import "$PROJECTS_ROOT/CANlib" \
    -import "$PROJECTS_ROOT/CoreN2G" \
    -import "$PROJECTS_ROOT/FreeRTOS" \
    -import "$PROJECTS_ROOT/LibMbedTls" \
    -import "$PROJECTS_ROOT/LibTinyusb" \
    -import "$PROJECTS_ROOT/Qfplib-M0-full" \
    -import "$PROJECTS_ROOT/RRFLibraries" \
    -import "$PROJECTS_ROOT/WiFiSocketServerRTOS" \
    -import "$PROJECTS_ROOT/RepRapFirmware" \
    -import "$PROJECTS_ROOT/Duet3Expansion"

echo "=== Building RepRapFirmware/Duet3Mini5plus ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "RepRapFirmware/Duet3Mini5plus"

echo "=== Building RepRapFirmware/Duet3_MB6HC ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "RepRapFirmware/Duet3_MB6HC"

echo "=== Building RepRapFirmware/Duet3_MB6XD ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "RepRapFirmware/Duet3_MB6XD"

echo "=== Building Duet3Expansion/EXP1HCL ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/EXP1HCL"

echo "=== Building Duet3Expansion/EXP1XD ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/EXP1XD"

echo "=== Building Duet3Expansion/EXP3HC ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/EXP3HC"

echo "=== Building Duet3Expansion/F3PTB ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/F3PTB"

echo "=== Building Duet3Expansion/M23CL ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/M23CL"

echo "=== Building Duet3Expansion/SAMMYC21 ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/SAMMYC21"

echo "=== Building Duet3Expansion/SZP ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/SZP"

echo "=== Building Duet3Expansion/TOOL1LC ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/TOOL1LC"

echo "=== Building Duet3Expansion/TOOL1RR ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/TOOL1RR"

echo "=== Building Duet3Expansion/TOOLINDX ==="
eclipse $ECLIPSE_ARGS -data "$WORKSPACE" -E ArmGccPath="$ARM_GCC_PATH" -cleanBuild "Duet3Expansion/TOOLINDX"

echo "=== Build complete ==="
