# RepRapFirmware Master Makefile
# Builds firmware for various Duet boards

# Cross-compiler toolchain (relative to project root)
#CROSS_COMPILE ?= ../arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
CROSS_COMPILE ?= ../arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
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

# Workspace root
WORKSPACE := ..
export WORKSPACE

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
CONFIGS := Duet2 Duet2_SBC Duet3_MB6HC Duet3_MB6XD Duet3_CAN0 Duet3Mini5plus Duet3_MB6HC_no_SD PCCB_10 FMDC_V03

# Print available targets
.PHONY: help
help:
	$(Q)echo ""
	$(Q)echo "RepRapFirmware Build System"
	$(Q)echo "============================"
	$(Q)echo ""
	$(Q)echo "Build targets:"
	$(Q)echo "  Duet2               - Duet 2 WiFi/Ethernet (SAM4E)"
	$(Q)echo "  Duet2_SBC           - Duet 2 + SBC (SAM4E)"
	$(Q)echo "  Duet3_MB6HC         - Duet 3 MB6HC (SAME70)"
	$(Q)echo "  Duet3_MB6XD         - Duet 3 MB6XD expansion (SAME70)"
	$(Q)echo "  Duet3_CAN0          - Duet 3 CAN expansion (SAME70)"
	$(Q)echo "  Duet3Mini5plus      - Duet 3 Mini 5+ (SAME51)"
	$(Q)echo "  PCCB_10             - PCCB version 1.0"
	$(Q)echo "  FMDC_V03            - FMDC version 0.3"
	$(Q)echo "  Duet3_MB6HC_no_SD   - Duet 3 MB6HC without SD card support"
	$(Q)echo ""
	$(Q)echo "Other targets:"
	$(Q)echo "  all                 - Build all configurations"
	$(Q)echo "  clean               - Clean all build outputs"
	$(Q)echo "  clean-all           - Clean all build outputs and libraries"
	$(Q)echo "  clean-<config>      - Clean specific configuration"
	$(Q)echo "  test                - Run host-native unit tests"
	$(Q)echo "  test-host           - Run host-native unit tests"
	$(Q)echo "  test-toolchain      - Verify toolchain is accessible"
	$(Q)echo ""
	$(Q)echo "Environment variables:"
	$(Q)echo "  CROSS_COMPILE       - Toolchain prefix (default: $(CROSS_COMPILE))"
	$(Q)echo "  V=1                 - Enable verbose build output"
	$(Q)echo "  DEBUG=1             - Build with debug symbols (-g3 -Og)"
	$(Q)echo ""
	$(Q)echo "Examples:"
	$(Q)echo "  make Duet2                                # Build Duet 2 firmware"
	$(Q)echo "  make Duet3Mini5plus V=1                   # Build with verbose output"
	$(Q)echo "  make Duet3_MB6HC DEBUG=1                  # Build with debug symbols"
	$(Q)echo "  make test-host                            # Run host-native unit tests"
	$(Q)echo "  make CROSS_COMPILE=/path/to/arm-none-eabi- Duet2  # Custom toolchain"
	$(Q)echo ""

# Build all configurations
.PHONY: all
all: $(CONFIGS)

# Verify toolchain
.PHONY: test test-host
test: test-host

test-host:
	$(Q)$(MAKE) $(VERBOSE) -C tests CXX=g++-15

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

# Build library dependencies
.PHONY: build-libs
build-libs:
	$(Q)echo "Building library dependencies..."
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/LibTinyusb

# Common library build rules (to avoid duplicate recipes in board makefiles)
# These are marked as .PHONY so Make always checks if they need rebuilding
.PHONY: $(WORKSPACE)/CoreN2G/SAM4E_SDHC_USB_RTOS/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAME5x_CAN_SDHC_USB_RTOS/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAME5x_SDHC_USB_RTOS/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAM4S_SDHC_USB_RTOS/libCoreN2G.a \
        $(WORKSPACE)/RRFLibraries/SAM4E_RTOS/libRRFLibraries.a \
        $(WORKSPACE)/RRFLibraries/SAME70_RTOS/libRRFLibraries.a \
        $(WORKSPACE)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a \
        $(WORKSPACE)/RRFLibraries/SAM4S_RTOS/libRRFLibraries.a \
        $(WORKSPACE)/FreeRTOS/SAM4E/libFreeRTOS.a \
        $(WORKSPACE)/FreeRTOS/SAME70/libFreeRTOS.a \
        $(WORKSPACE)/FreeRTOS/SAME51/libFreeRTOS.a \
        $(WORKSPACE)/FreeRTOS/SAM4S/libFreeRTOS.a \
        $(WORKSPACE)/CANlib/SAM4E_RTOS/libCANlib.a \
        $(WORKSPACE)/CANlib/SAME70_RTOS/libCANlib.a \
        $(WORKSPACE)/CANlib/SAME51_RTOS/libCANlib.a \
        $(WORKSPACE)/CANlib/SAM4S_RTOS/libCANlib.a \
        $(WORKSPACE)/LibTinyusb/SAME70/libLibTinyusb.a \
        $(WORKSPACE)/LibTinyusb/SAME5x/libLibTinyusb.a

$(WORKSPACE)/CoreN2G/SAM4E_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAM4E_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAM4E_SDHC_USB_RTOS

$(WORKSPACE)/RRFLibraries/SAM4E_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAM4E_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAM4E_RTOS

$(WORKSPACE)/FreeRTOS/SAM4E/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAM4E"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS SAM4E FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(WORKSPACE)/CANlib/SAM4E_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAM4E_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAM4E_RTOS

$(WORKSPACE)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME70_CAN_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME70_CAN_SDHC_USB_RTOS

$(WORKSPACE)/RRFLibraries/SAME70_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME70_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAME70_RTOS

$(WORKSPACE)/FreeRTOS/SAME70/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAME70"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS SAME70 FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(WORKSPACE)/CANlib/SAME70_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAME70_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAME70_RTOS

$(WORKSPACE)/LibTinyusb/SAME70/libLibTinyusb.a:
	$(Q)echo "  BUILD   LibTinyusb/SAME70"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/LibTinyusb SAME70

$(WORKSPACE)/CoreN2G/SAME5x_CAN_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_CAN_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME5x_CAN_SDHC_USB_RTOS

$(WORKSPACE)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME51_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAME51_RTOS

$(WORKSPACE)/FreeRTOS/SAME51/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAME51"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS SAME51 FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(WORKSPACE)/CANlib/SAME51_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAME51_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAME51_RTOS

$(WORKSPACE)/LibTinyusb/SAME5x/libLibTinyusb.a:
	$(Q)echo "  BUILD   LibTinyusb/SAME5x"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/LibTinyusb SAME5x

$(WORKSPACE)/CoreN2G/SAME5x_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME5x_SDHC_USB_RTOS

$(WORKSPACE)/CoreN2G/SAM4S_SDHC_USB_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAM4S_SDHC_USB_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAM4S_SDHC_USB_RTOS

$(WORKSPACE)/RRFLibraries/SAM4S_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAM4S_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAM4S_RTOS

$(WORKSPACE)/FreeRTOS/SAM4S/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAM4S"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS SAM4S FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(WORKSPACE)/CANlib/SAM4S_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAM4S_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAM4S_RTOS

# Include dependency makefiles
-include Makefiles/Duet2.mk
-include Makefiles/Duet2_SBC.mk
-include Makefiles/Duet3_MB6HC.mk
-include Makefiles/Duet3_MB6XD.mk
-include Makefiles/Duet3_CAN0.mk
-include Makefiles/Duet3Mini5plus.mk
-include Makefiles/PCCB_10.mk
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
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/LibTinyusb clean
	$(Q)echo "Clean all complete"
