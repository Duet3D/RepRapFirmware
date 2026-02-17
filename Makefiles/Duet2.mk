# RepRapFirmware Duet2 Configuration Makefile
# Build configuration for Duet 2 WiFi/Ethernet boards

# Build directory and output
DUET2_BUILD_DIR := Duet2
DUET2_TARGET_NAME := Duet2CombinedFirmware
DUET2_TARGET_ELF := $(DUET2_BUILD_DIR)/$(DUET2_TARGET_NAME).elf
DUET2_TARGET_BIN := $(DUET2_BUILD_DIR)/$(DUET2_TARGET_NAME).bin
DUET2_TARGET_MAP := $(DUET2_BUILD_DIR)/$(DUET2_TARGET_NAME).map

# Workspace root (relative paths from RepRapFirmware directory)
WORKSPACE := ..

# Library dependencies
DUET2_FREERTOS_LIB := $(WORKSPACE)/FreeRTOS/SAM4E/libFreeRTOS.a
DUET2_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAM4E_SDHC_USB_RTOS/libCoreN2G.a
DUET2_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAM4E_RTOS/libRRFLibraries.a
DUET2_CANLIB_LIB := $(WORKSPACE)/CANlib/SAM4E_RTOS/libCANlib.a

# Source directories
DUET2_SRC_DIR := src

# Find libc and libcpp files first (must be linked first to avoid malloc conflicts)
DUET2_LIBCPP_SRCS := $(shell find $(DUET2_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
DUET2_LIBC_SRCS := $(shell find $(DUET2_SRC_DIR)/libc -name '*.c' 2>/dev/null)

# Find all other source files (excluding specified directories and libc/libcpp already found)
DUET2_OTHER_CPP_SRCS := $(shell find $(DUET2_SRC_DIR) -name '*.cpp' \
	! -path '*/SBC/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Hardware/ksz8081rna/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Pccb/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/libcpp/*')

DUET2_OTHER_C_SRCS := $(shell find $(DUET2_SRC_DIR) -name '*.c' \
	! -path '*/SBC/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Hardware/ksz8081rna/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Pccb/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/MQTT_C/tests.c' \
	! -path '*/MQTT_C/examples/*' \
	! -path '*/MQTT_C/src/mqtt_pal.c' \
	! -path '*/libc/*')

# Combine with libcpp and libc first
DUET2_CPP_SRCS := $(DUET2_LIBCPP_SRCS) $(DUET2_OTHER_CPP_SRCS)
DUET2_C_SRCS := $(DUET2_LIBC_SRCS) $(DUET2_OTHER_C_SRCS)

# Include paths
DUET2_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4E \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/FreeRTOS \
	-I$(WORKSPACE)/FreeRTOS/src/include \
	-I$(WORKSPACE)/FreeRTOS/src/portable/GCC/ARM_CM4F \
	-I$(WORKSPACE)/RRFLibraries/src \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/WiFiSocketServerRTOS/src/include \
	-I$(DUET2_SRC_DIR) \
	-I$(DUET2_SRC_DIR)/Hardware/SAM4E \
	-I$(DUET2_SRC_DIR)/DuetNG \
	-I$(DUET2_SRC_DIR)/Networking \
	-I$(DUET2_SRC_DIR)/Networking/MQTT/MQTT_C/include

# Preprocessor defines
# Note: MQTTC_PAL_FILE needs special handling for quotes
MQTTC_PAL_DEFINE := Networking/MQTT/mqtt_pal.h
DUET2_DEFINES := \
	-D__SAM4E8E__ \
	-DRTOS \
	-DDUET_NG \
	-D_XOPEN_SOURCE

# Compiler flags - C
DUET2_CFLAGS := -c -std=gnu99 \
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
	$(DUET2_INCLUDES) \
	$(DUET2_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
DUET2_CXXFLAGS := -c -std=gnu++17 \
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
	-Wsign-promo \
	$(DUET2_INCLUDES) \
	$(DUET2_DEFINES)

# Library search paths
DUET2_LIBPATHS := \
	-L$(WORKSPACE)/CANlib/SAM4E_RTOS \
	-L$(WORKSPACE)/CoreN2G/SAM4E_SDHC_USB_RTOS \
	-L$(WORKSPACE)/RRFLibraries/SAM4E_RTOS \
	-L$(WORKSPACE)/FreeRTOS/SAM4E

# Linker flags (part 1 - before output)
DUET2_LDFLAGS1 := $(DUET2_LIBPATHS) \
	--specs=nosys.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(DUET2_SRC_DIR)/Hardware/SAM4E/sam4e8e_flash.ld \
	-Wl,-Map,$(DUET2_TARGET_MAP)

# Linker flags (part 2 - after output, before objects)
DUET2_LDFLAGS2 := \
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
DUET2_LDLIBS := \
	-lCANlib \
	-lCoreN2G \
	-lRRFLibraries \
	-lFreeRTOS \
	-lsupc++

# Libraries after end-group
DUET2_LDLIBS_POST := -Wl,--end-group -lm

# Object files
DUET2_CPP_OBJS := $(DUET2_CPP_SRCS:%.cpp=$(DUET2_BUILD_DIR)/%.o)
DUET2_CPP_OBJS := $(DUET2_CPP_OBJS:%.cc=$(DUET2_BUILD_DIR)/%.o)
DUET2_C_OBJS := $(DUET2_C_SRCS:%.c=$(DUET2_BUILD_DIR)/%.o)
DUET2_OBJS := $(DUET2_CPP_OBJS) $(DUET2_C_OBJS)

# Dependency files
DUET2_DEPS := $(DUET2_OBJS:.o=.d)

# Target rule
.PHONY: Duet2
Duet2: $(DUET2_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet2 firmware build complete!"
	$(Q)echo "Output: $(DUET2_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(DUET2_TARGET_ELF)

# Link ELF file
$(DUET2_TARGET_ELF): $(DUET2_OBJS) $(DUET2_CANLIB_LIB) $(DUET2_COREN2G_LIB) $(DUET2_RRFLIBS_LIB) $(DUET2_FREERTOS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(DUET2_LDFLAGS1) -o $@ $(DUET2_LDFLAGS2) $(DUET2_OBJS) $(DUET2_LDLIBS) $(DUET2_LDLIBS_POST)

# Generate binary file
$(DUET2_TARGET_BIN): $(DUET2_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)if command -v CrcAppender >/dev/null 2>&1; then \
		echo "  CRC     $@"; \
		CrcAppender $@; \
	fi

# Compile C++ files
$(DUET2_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET2_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(DUET2_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET2_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile C files
$(DUET2_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(DUET2_CFLAGS) -Dnoexcept= -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Touch Version.cpp before build (pre-build step)
.PHONY: duet2-prebuild
duet2-prebuild:
	$(Q)touch -c $(DUET2_SRC_DIR)/Version.cpp 2>/dev/null || true

$(DUET2_OBJS): | duet2-prebuild

# Include dependencies
-include $(DUET2_DEPS)

# Clean target
.PHONY: clean-Duet2
clean-Duet2:
	$(Q)echo "  RM      $(DUET2_BUILD_DIR)"
	$(Q)rm -rf $(DUET2_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: duet2-libs
duet2-libs: $(DUET2_FREERTOS_LIB) $(DUET2_COREN2G_LIB) $(DUET2_RRFLIBS_LIB)
	$(Q)echo "All required libraries built successfully"

