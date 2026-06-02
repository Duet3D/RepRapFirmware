# RepRapFirmware FMDC_V03 Configuration Makefile
# Build configuration for FMDC (Flexible Motor Drive Controller) Version 0.3
# Hardware: SAME51N19A (Cortex-M4F)

# Build directory and output
FMDC_V03_BUILD_DIR := FMDC_V03
FMDC_V03_TARGET_NAME := Duet3Firmware_FMDC
FMDC_V03_TARGET_ELF := $(FMDC_V03_BUILD_DIR)/$(FMDC_V03_TARGET_NAME).elf
FMDC_V03_TARGET_BIN := $(FMDC_V03_BUILD_DIR)/$(FMDC_V03_TARGET_NAME).bin
FMDC_V03_TARGET_UF2 := $(FMDC_V03_BUILD_DIR)/$(FMDC_V03_TARGET_NAME).uf2
FMDC_V03_TARGET_MAP := $(FMDC_V03_BUILD_DIR)/$(FMDC_V03_TARGET_NAME).map

# External library root
LIBRARIES_DIR ?= libraries

# Library dependencies
FMDC_V03_FREERTOS_LIB := $(LIBRARIES_DIR)/FreeRTOS/SAME51/libFreeRTOS.a
FMDC_V03_COREN2G_LIB := $(LIBRARIES_DIR)/CoreN2G/SAME5x_SDHC_USB_RTOS/libCoreN2G.a
FMDC_V03_RRFLIBS_LIB := $(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a
FMDC_V03_CANLIB_LIB := $(LIBRARIES_DIR)/CANlib/SAME51_RTOS/libCANlib.a
FMDC_V03_LIBTINYUSB_LIB := $(LIBRARIES_DIR)/LibTinyusb/SAME5x/libLibTinyusb.a

# Source directories
FMDC_V03_SRC_DIR := src

# Find libc and libcpp files first (must be linked first to avoid malloc conflicts)
FMDC_V03_LIBCPP_SRCS := $(shell find $(FMDC_V03_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
FMDC_V03_LIBC_SRCS := $(shell find $(FMDC_V03_SRC_DIR)/libc -name '*.c' 2>/dev/null)

# Find all other source files (excluding specified directories and libc/libcpp already found)
# Exclusions based on .cproject FMDC_V03 configuration
FMDC_V03_OTHER_CPP_SRCS := $(shell find $(FMDC_V03_SRC_DIR) -name '*.cpp' \
	! -path '*/Hardware/SAME5x/Ethernet/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Networking/W5500Ethernet/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/Pccb/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/libcpp/*')

FMDC_V03_OTHER_C_SRCS := $(shell find $(FMDC_V03_SRC_DIR) -name '*.c' \
	! -path '*/Hardware/SAME5x/Ethernet/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Networking/W5500Ethernet/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/Pccb/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/MQTT_C/tests.c' \
	! -path '*/MQTT_C/examples/*' \
	! -path '*/MQTT_C/src/mqtt_pal.c' \
	! -path '*/libc/*')

# Combine with libcpp and libc first
FMDC_V03_CPP_SRCS := $(FMDC_V03_LIBCPP_SRCS) $(FMDC_V03_OTHER_CPP_SRCS)
FMDC_V03_C_SRCS := $(FMDC_V03_LIBC_SRCS) $(FMDC_V03_OTHER_C_SRCS)

# Include paths
FMDC_V03_INCLUDES := \
	-I$(LIBRARIES_DIR)/CoreN2G \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21 \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hal/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hri \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/Config \
	-I$(LIBRARIES_DIR)/CoreN2G/src/atmel/SAME51_DFP/1.1.139/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(LIBRARIES_DIR)/FreeRTOS \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/include \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/portable/GCC/ARM_CM4F \
	-I$(LIBRARIES_DIR)/RRFLibraries/src \
	-I$(LIBRARIES_DIR)/LibTinyusb \
	-I$(LIBRARIES_DIR)/CANlib/src \
	-I$(LIBRARIES_DIR)/WiFiSocketServerRTOS/src/include \
	-I$(FMDC_V03_SRC_DIR) \
	-I$(FMDC_V03_SRC_DIR)/Hardware/SAME5x \
	-I$(FMDC_V03_SRC_DIR)/Networking \
	-I$(FMDC_V03_SRC_DIR)/Networking/MQTT/MQTT_C/include

# Preprocessor defines
# Note: MQTTC_PAL_FILE needs special handling for quotes
MQTTC_PAL_DEFINE := Networking/MQTT/mqtt_pal.h
FMDC_V03_DEFINES := \
	-D__SAME51N19A__ \
	-DRTOS \
	-DFMDC_V03

# Compiler flags - C
FMDC_V03_CFLAGS := -c -std=gnu99 \
	-Wall \
	-mcpu=cortex-m4 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-Werror=return-type \
	-Werror=implicit \
	-fsingle-precision-constant \
	-Os \
	-Werror \
	-Wwrite-strings \
	$(FMDC_V03_INCLUDES) \
	$(FMDC_V03_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
FMDC_V03_CXXFLAGS := -c -std=c++20 \
	-Wall \
	-mcpu=cortex-m4 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-fno-threadsafe-statics \
	-fno-rtti \
	-fexceptions \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-Wfloat-conversion \
	-Werror=return-type \
	-Wsuggest-override \
	-fsingle-precision-constant \
	-fstack-usage \
	-Os \
	-Werror \
	-Wnoexcept \
	-Wshadow \
	-Woverloaded-virtual \
	$(FMDC_V03_INCLUDES) \
	$(FMDC_V03_DEFINES) \
	-D_XOPEN_SOURCE

# Library search paths
FMDC_V03_LIBPATHS := \
	-L$(LIBRARIES_DIR)/LibTinyusb/SAME5x \
	-L$(LIBRARIES_DIR)/CANlib/SAME51_RTOS \
	-L$(LIBRARIES_DIR)/CoreN2G/SAME5x_SDHC_USB_RTOS \
	-L$(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS \
	-L$(LIBRARIES_DIR)/FreeRTOS/SAME51

# Linker flags (part 1 - before output)
FMDC_V03_LDFLAGS1 := $(FMDC_V03_LIBPATHS) \
	--specs=nosys.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(FMDC_V03_SRC_DIR)/Hardware/SAME5x/same51n19a_flash_16k_bootloader.ld \
	-Wl,-Map,$(FMDC_V03_TARGET_MAP)

# Linker flags (part 2 - after output, before objects)
FMDC_V03_LDFLAGS2 := \
	-mthumb \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols \
	-Wl,--start-group

# Libraries (between start-group and end-group)
FMDC_V03_LDLIBS := \
	-lLibTinyusb \
	-lCANlib \
	-lCoreN2G \
	-lRRFLibraries \
	-lFreeRTOS \
	-lsupc++

# Libraries after end-group
FMDC_V03_LDLIBS_POST := -Wl,--end-group -lm

# Object files
FMDC_V03_CPP_OBJS := $(FMDC_V03_CPP_SRCS:%.cpp=$(FMDC_V03_BUILD_DIR)/%.o)
FMDC_V03_CPP_OBJS := $(FMDC_V03_CPP_OBJS:%.cc=$(FMDC_V03_BUILD_DIR)/%.o)
FMDC_V03_C_OBJS := $(FMDC_V03_C_SRCS:%.c=$(FMDC_V03_BUILD_DIR)/%.o)
FMDC_V03_OBJS := $(FMDC_V03_CPP_OBJS) $(FMDC_V03_C_OBJS)

# Dependency files
FMDC_V03_DEPS := $(FMDC_V03_OBJS:.o=.d)

# Target rule
.PHONY: FMDC_V03
FMDC_V03: $(FMDC_V03_TARGET_UF2)
	$(Q)echo "========================================"
	$(Q)echo "FMDC_V03 firmware build complete!"
	$(Q)echo "Output: $(FMDC_V03_TARGET_BIN)"
	$(Q)echo "UF2: $(FMDC_V03_TARGET_UF2)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(FMDC_V03_TARGET_ELF)

# Link ELF file
$(FMDC_V03_TARGET_ELF): $(FMDC_V03_OBJS) $(FMDC_V03_CANLIB_LIB) $(FMDC_V03_COREN2G_LIB) $(FMDC_V03_RRFLIBS_LIB) $(FMDC_V03_FREERTOS_LIB) $(FMDC_V03_LIBTINYUSB_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(FMDC_V03_LDFLAGS1) -o $@ $(FMDC_V03_LDFLAGS2) $(FMDC_V03_OBJS) $(FMDC_V03_LDLIBS) $(FMDC_V03_LDLIBS_POST)

# Generate binary file
$(FMDC_V03_TARGET_BIN): $(FMDC_V03_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)command -v CrcAppender >/dev/null 2>&1 || { echo "CrcAppender not found on PATH" >&2; exit 1; }
	$(Q)echo "  CRC     $@"
	$(Q)CrcAppender $@

# Generate UF2 file for USB bootloader
$(FMDC_V03_TARGET_UF2): $(FMDC_V03_TARGET_BIN)
	$(Q)echo "  UF2     $@"
	$(Q)if [ -f Tools/uf2conv/uf2conv.py ]; then \
		python3 Tools/uf2conv/uf2conv.py -b 0x4000 -c -o $@ $<; \
	else \
		echo "  WARNING: uf2conv.py not found, skipping UF2 generation"; \
	fi

# Compile C++ files
$(FMDC_V03_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(FMDC_V03_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(FMDC_V03_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(FMDC_V03_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile C files
$(FMDC_V03_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(FMDC_V03_CFLAGS) -Dnoexcept= -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Touch Version.cpp before build (pre-build step)
.PHONY: fmdc_v03-prebuild
fmdc_v03-prebuild:
	$(Q)touch -c $(FMDC_V03_SRC_DIR)/Version.cpp 2>/dev/null || true

$(FMDC_V03_OBJS): | fmdc_v03-prebuild

# Include dependencies
-include $(FMDC_V03_DEPS)

# Clean target
.PHONY: clean-FMDC_V03
clean-FMDC_V03:
	$(Q)echo "  RM      $(FMDC_V03_BUILD_DIR)"
	$(Q)rm -rf $(FMDC_V03_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: fmdc_v03-libs
fmdc_v03-libs: $(FMDC_V03_FREERTOS_LIB) $(FMDC_V03_COREN2G_LIB) $(FMDC_V03_RRFLIBS_LIB) $(FMDC_V03_CANLIB_LIB) $(FMDC_V03_LIBTINYUSB_LIB)
	$(Q)echo "All required libraries built successfully"
