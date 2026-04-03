# RepRapFirmware Master Makefile
# Builds firmware for various Duet boards

# Cross-compiler toolchain (relative to project root)
ARM_GNU_TOOLCHAIN_VERSION ?= 15.2.rel1
HOST_OS_RAW := $(shell uname -s)
HOST_ARCH_RAW := $(shell uname -m)

ifeq ($(HOST_OS_RAW),Linux)
HOST_OS := linux
else ifeq ($(HOST_OS_RAW),Darwin)
HOST_OS := macos
else
HOST_OS := $(HOST_OS_RAW)
endif

ifeq ($(HOST_ARCH_RAW),aarch64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),arm64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),x86_64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else ifeq ($(HOST_ARCH_RAW),amd64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else
ARM_GNU_TOOLCHAIN_HOST_ARCH := $(HOST_ARCH_RAW)
endif

CRC_APPENDER_DIR := $(abspath Tools/CrcAppender/$(HOST_OS)-$(ARM_GNU_TOOLCHAIN_HOST_ARCH))
ifneq ($(wildcard $(CRC_APPENDER_DIR)/CrcAppender),)
export PATH := $(CRC_APPENDER_DIR):$(PATH)
endif

CROSS_COMPILE ?= $(abspath ../arm-gnu-toolchain-$(ARM_GNU_TOOLCHAIN_VERSION)-$(ARM_GNU_TOOLCHAIN_HOST_ARCH)-arm-none-eabi/bin/arm-none-eabi-)
export CROSS_COMPILE

# Toolchain programs
CC  := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
AS  := $(CROSS_COMPILE)gcc
AR  := $(CROSS_COMPILE)ar
LD  := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE := $(CROSS_COMPILE)size
export CC CXX AS AR LD OBJCOPY SIZE

# External library root
LIBRARIES_DIR ?= libraries

# Quiet build support (Linux kernel style)
# Use V=1 for verbose output
ifeq ($(V),1)
	Q :=
	VERBOSE :=
else
	Q := @
	VERBOSE := -s
endif
export Q VERBOSE

# Debug build support
# Use DEBUG=1 to build with debug symbols and reduced optimization
ifeq ($(DEBUG),1)
DEBUG_FLAGS := -g3 -Og -DDEBUG
$(info Building with debug symbols enabled)
else
DEBUG_FLAGS :=
endif
export DEBUG_FLAGS

# Default target
.DEFAULT_GOAL := help

# Available build configurations
CONFIGS := Duet3_MB6HC Duet3_MB6XD Duet3_CAN0 Duet3Mini5plus Duet3_MB6HC_no_SD FMDC_V03

# Print available targets
.PHONY: help
help:
	$(Q)echo ""
	$(Q)echo "RepRapFirmware Build System"
	$(Q)echo "============================"
	$(Q)echo ""
	$(Q)echo "Build targets:"
	$(Q)echo "  Duet3_MB6HC         - Duet 3 MB6HC (SAME70)"
	$(Q)echo "  Duet3_MB6XD         - Duet 3 MB6XD expansion (SAME70)"
	$(Q)echo "  Duet3_CAN0          - Duet 3 CAN expansion (SAME70)"
	$(Q)echo "  Duet3Mini5plus      - Duet 3 Mini 5+ (SAME51)"
	$(Q)echo "  FMDC_V03            - FMDC version 0.3"
	$(Q)echo "  Duet3_MB6HC_no_SD   - Duet 3 MB6HC without SD card support"
	$(Q)echo ""
	$(Q)echo "Other targets:"
	$(Q)echo "  all                 - Build all configurations"
	$(Q)echo "  init-submodules     - Initialize/update pinned library submodules"
	$(Q)echo "  clean               - Clean all build outputs"
	$(Q)echo "  clean-all           - Clean all build outputs and libraries"
	$(Q)echo "  clean-<config>      - Clean specific configuration"
	$(Q)echo "  test-toolchain      - Verify toolchain is accessible"
	$(Q)echo ""
	$(Q)echo "Environment variables:"
	$(Q)echo "  ARM_GNU_TOOLCHAIN_VERSION - Toolchain version (default: $(ARM_GNU_TOOLCHAIN_VERSION))"
	$(Q)echo "  CROSS_COMPILE       - Toolchain prefix (default: $(CROSS_COMPILE))"
	$(Q)echo "  V=1                 - Enable verbose build output"
	$(Q)echo "  DEBUG=1             - Build with debug symbols (-g3 -Og)"
	$(Q)echo ""
	$(Q)echo "Examples:"
	$(Q)echo "  make init-submodules                      # Prepare library submodules after clone"
	$(Q)echo "  make Duet3_MB6HC                          # Build Duet 3 MB6HC firmware"
	$(Q)echo "  make Duet3Mini5plus V=1                   # Build with verbose output"
	$(Q)echo "  make Duet3_MB6HC DEBUG=1                  # Build with debug symbols"
	$(Q)echo "  make CROSS_COMPILE=/path/to/arm-none-eabi- Duet3_MB6HC  # Custom toolchain"
	$(Q)echo ""

# Build all configurations
.PHONY: all
all: $(CONFIGS)

# Verify toolchain
.PHONY: test-toolchain
test-toolchain:
	$(Q)echo "Testing toolchain..."
	$(Q)if [ ! -f "$(CROSS_COMPILE)gcc" ]; then \
		echo "ERROR: Toolchain not found at: $(CROSS_COMPILE)gcc"; \
		echo "Please install the ARM GCC toolchain and set CROSS_COMPILE"; \
		exit 1; \
	fi
	$(Q)echo "Toolchain: $(CROSS_COMPILE)"
	$(Q)$(CROSS_COMPILE)gcc --version | head -n 1
	$(Q)echo "Toolchain OK"

SUBMODULE_PATHS := \
	$(LIBRARIES_DIR)/CANlib \
	$(LIBRARIES_DIR)/CoreN2G \
	$(LIBRARIES_DIR)/FreeRTOS \
	$(LIBRARIES_DIR)/RRFLibraries \
	$(LIBRARIES_DIR)/WiFiSocketServerRTOS \
	$(LIBRARIES_DIR)/LibTinyusb \
	$(LIBRARIES_DIR)/LibMbedTls

LIBRARY_ARTIFACTS := \
	$(LIBRARIES_DIR)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS/libCoreN2G.a \
	$(LIBRARIES_DIR)/CoreN2G/SAME5x_CAN_SDHC_USB_RTOS/libCoreN2G.a \
	$(LIBRARIES_DIR)/CoreN2G/SAME5x_SDHC_USB_RTOS/libCoreN2G.a \
	$(LIBRARIES_DIR)/CoreN2G/SAM4S_SDHC_USB_RTOS/libCoreN2G.a \
	$(LIBRARIES_DIR)/RRFLibraries/SAME70_RTOS/libRRFLibraries.a \
	$(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a \
	$(LIBRARIES_DIR)/RRFLibraries/SAM4S_RTOS/libRRFLibraries.a \
	$(LIBRARIES_DIR)/FreeRTOS/SAME70/libFreeRTOS.a \
	$(LIBRARIES_DIR)/FreeRTOS/SAME51/libFreeRTOS.a \
	$(LIBRARIES_DIR)/FreeRTOS/SAM4S/libFreeRTOS.a \
	$(LIBRARIES_DIR)/CANlib/SAME70_RTOS/libCANlib.a \
	$(LIBRARIES_DIR)/CANlib/SAME51_RTOS/libCANlib.a \
	$(LIBRARIES_DIR)/CANlib/SAM4S_RTOS/libCANlib.a \
	$(LIBRARIES_DIR)/LibTinyusb/SAME70/libLibTinyusb.a \
	$(LIBRARIES_DIR)/LibTinyusb/SAME5x/libLibTinyusb.a \
	$(LIBRARIES_DIR)/LibMbedTls/SAME70/libLibMbedTls.a \
	$(LIBRARIES_DIR)/LibMbedTls/SAME5x/libLibMbedTls.a

# Initialize library submodules, including nested submodules such as LibTinyusb/src/tinyusb.
# Keep this explicit so normal builds do not disturb local work inside the submodules.
.PHONY: init-submodules
init-submodules:
	$(Q)echo "Initializing library submodules..."
	$(Q)git submodule update --init --recursive -- $(SUBMODULE_PATHS)

# Build library dependencies. Assumes submodules have already been initialized.
.PHONY: build-libs
build-libs: $(LIBRARY_ARTIFACTS)
	$(Q)echo "Building library dependencies..."
	$(Q)echo "Library dependencies built successfully"

# Common library build rules (to avoid duplicate recipes in board makefiles)
# These are marked as .PHONY so Make always checks if they need rebuilding
.PHONY: $(LIBRARY_ARTIFACTS)

$(LIBRARIES_DIR)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME70_CAN_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CoreN2G SAME70_CAN_SDHC_USB_RTOS

$(LIBRARIES_DIR)/RRFLibraries/SAME70_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME70_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/RRFLibraries SAME70_RTOS

$(LIBRARIES_DIR)/FreeRTOS/SAME70/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAME70"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/FreeRTOS SAME70 FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(LIBRARIES_DIR)/CANlib/SAME70_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAME70_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CANlib SAME70_RTOS

$(LIBRARIES_DIR)/LibTinyusb/SAME70/libLibTinyusb.a:
	$(Q)echo "  BUILD   LibTinyusb/SAME70"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/LibTinyusb SAME70

$(LIBRARIES_DIR)/CoreN2G/SAME5x_CAN_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_CAN_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CoreN2G SAME5x_CAN_SDHC_USB_RTOS

$(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME51_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/RRFLibraries SAME51_RTOS

$(LIBRARIES_DIR)/FreeRTOS/SAME51/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAME51"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/FreeRTOS SAME51 FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(LIBRARIES_DIR)/CANlib/SAME51_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAME51_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CANlib SAME51_RTOS

$(LIBRARIES_DIR)/LibTinyusb/SAME5x/libLibTinyusb.a:
	$(Q)echo "  BUILD   LibTinyusb/SAME5x"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/LibTinyusb SAME5x

$(LIBRARIES_DIR)/LibMbedTls/SAME70/libLibMbedTls.a:
	$(Q)echo "  BUILD   LibMbedTls/SAME70"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/LibMbedTls SAME70

$(LIBRARIES_DIR)/LibMbedTls/SAME5x/libLibMbedTls.a:
	$(Q)echo "  BUILD   LibMbedTls/SAME5x"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/LibMbedTls SAME5x

$(LIBRARIES_DIR)/CoreN2G/SAME5x_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CoreN2G SAME5x_SDHC_USB_RTOS

$(LIBRARIES_DIR)/CoreN2G/SAM4S_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAM4S_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CoreN2G SAM4S_SDHC_USB_RTOS

$(LIBRARIES_DIR)/RRFLibraries/SAM4S_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAM4S_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/RRFLibraries SAM4S_RTOS

$(LIBRARIES_DIR)/FreeRTOS/SAM4S/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAM4S"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/FreeRTOS SAM4S FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(LIBRARIES_DIR)/CANlib/SAM4S_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAM4S_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CANlib SAM4S_RTOS

# Include dependency makefiles
-include Makefiles/Duet3_MB6HC.mk
-include Makefiles/Duet3_MB6XD.mk
-include Makefiles/Duet3_CAN0.mk
-include Makefiles/Duet3Mini5plus.mk
-include Makefiles/FMDC_V03.mk
-include Makefiles/Duet3_MB6HC_no_SD.mk

# Generic clean target
.PHONY: clean
clean:
	$(Q)echo "Cleaning all build outputs..."
	$(Q)for config in $(CONFIGS); do \
		if [ -d "$$config" ]; then \
			echo "  RM      $$config"; \
			rm -rf "$$config"; \
		fi; \
	done
	$(Q)echo "Clean complete"

# Clean all including libraries
.PHONY: clean-all
clean-all: clean
	$(Q)echo "Cleaning library dependencies..."
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/FreeRTOS clean
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CoreN2G clean
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/RRFLibraries clean
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/CANlib clean
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/LibTinyusb clean
	$(Q)$(MAKE) $(VERBOSE) -C $(LIBRARIES_DIR)/LibMbedTls clean
	$(Q)echo "Clean all complete"
