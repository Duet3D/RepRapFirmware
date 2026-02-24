# RepRapFirmware PCCB_10 Configuration Makefile
# Build configuration for PCCB (Printer Control Circuit Board) Version 1.0
# Hardware: SAM4S8C (Cortex-M4)

# Build directory and output
PCCB_10_BUILD_DIR := PCCB_10
PCCB_10_TARGET_NAME := PccbFirmware
PCCB_10_TARGET_ELF := $(PCCB_10_BUILD_DIR)/$(PCCB_10_TARGET_NAME).elf
PCCB_10_TARGET_BIN := $(PCCB_10_BUILD_DIR)/$(PCCB_10_TARGET_NAME).bin
PCCB_10_TARGET_MAP := $(PCCB_10_BUILD_DIR)/$(PCCB_10_TARGET_NAME).map

# Workspace root (relative paths from RepRapFirmware directory)
WORKSPACE := ..

# Library dependencies
PCCB_10_FREERTOS_LIB := $(WORKSPACE)/FreeRTOS/SAM4S/libFreeRTOS.a
PCCB_10_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAM4S_SDHC_USB_RTOS/libCoreN2G.a
PCCB_10_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAM4S_RTOS/libRRFLibraries.a
PCCB_10_CANLIB_LIB := $(WORKSPACE)/CANlib/SAM4S_RTOS/libCANlib.a

# Source directories
PCCB_10_SRC_DIR := src

# Find libc and libcpp files first (must be linked first to avoid malloc conflicts)
PCCB_10_LIBCPP_SRCS := $(shell find $(PCCB_10_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
PCCB_10_LIBC_SRCS := $(shell find $(PCCB_10_SRC_DIR)/libc -name '*.c' 2>/dev/null)

# Find all other source files (excluding specified directories and libc/libcpp already found)
PCCB_10_OTHER_CPP_SRCS := $(shell find $(PCCB_10_SRC_DIR) -name '*.cpp' \
	! -path '*/SBC/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Hardware/ksz8081rna/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Networking/W5500Ethernet/*' \
	! -path '*/Networking/ESP8266WiFi/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Networking/MQTT/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/DuetM/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Display/*' \
	! -path '*/libcpp/*')

PCCB_10_OTHER_C_SRCS := $(shell find $(PCCB_10_SRC_DIR) -name '*.c' \
	! -path '*/SBC/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Hardware/ksz8081rna/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Networking/W5500Ethernet/*' \
	! -path '*/Networking/ESP8266WiFi/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Networking/MQTT/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/DuetM/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Display/*' \
	! -path '*/libc/*')

# Combine with libcpp and libc first
PCCB_10_CPP_SRCS := $(PCCB_10_LIBCPP_SRCS) $(PCCB_10_OTHER_CPP_SRCS)
PCCB_10_C_SRCS := $(PCCB_10_LIBC_SRCS) $(PCCB_10_OTHER_C_SRCS)

# Include paths
PCCB_10_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4S \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/FreeRTOS \
	-I$(WORKSPACE)/FreeRTOS/src/include \
	-I$(WORKSPACE)/FreeRTOS/src/portable/GCC/ARM_CM3 \
	-I$(WORKSPACE)/RRFLibraries/src \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/WiFiSocketServerRTOS/src/include \
	-I$(PCCB_10_SRC_DIR) \
	-I$(PCCB_10_SRC_DIR)/Hardware/SAM4S

# Preprocessor defines
PCCB_10_DEFINES := \
	-D__SAM4S8C__ \
	-DRTOS \
	-DPCCB \
	-DPCCB_10 \
	-D_XOPEN_SOURCE

# Compiler flags - C
PCCB_10_CFLAGS := -c -std=gnu99 \
	-Wall \
	-mcpu=cortex-m4 \
	-mthumb \
	-fno-math-errno \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-Werror=return-type \
	-Werror=implicit \
	-fsingle-precision-constant \
	-O2 \
	-Werror \
	-Wwrite-strings \
	$(PCCB_10_INCLUDES) \
	$(PCCB_10_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
PCCB_10_CXXFLAGS := -c -std=gnu++17 \
	-Wall \
	-mcpu=cortex-m4 \
	-mthumb \
	-fno-math-errno \
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
	-O2 \
	-Werror \
	-Wnoexcept \
	-Wshadow \
	-Wsign-promo \
	$(PCCB_10_INCLUDES) \
	$(PCCB_10_DEFINES) \
	$(DEBUG_FLAGS)

# Library search paths
PCCB_10_LIBPATHS := \
	-L$(WORKSPACE)/CANlib/SAM4S_RTOS \
	-L$(WORKSPACE)/CoreN2G/SAM4S_SDHC_USB_RTOS \
	-L$(WORKSPACE)/RRFLibraries/SAM4S_RTOS \
	-L$(WORKSPACE)/FreeRTOS/SAM4S

# Linker flags (part 1 - before output)
PCCB_10_LDFLAGS1 := $(PCCB_10_LIBPATHS) \
	--specs=nosys.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m4 \
	-T$(PCCB_10_SRC_DIR)/Hardware/SAM4S/sam4s8c_flash.ld \
	-Wl,-Map,$(PCCB_10_TARGET_MAP)

# Linker flags (part 2 - after output, before objects)
PCCB_10_LDFLAGS2 := \
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
PCCB_10_LDLIBS := \
	-lCANlib \
	-lCoreN2G \
	-lRRFLibraries \
	-lFreeRTOS \
	-lsupc++

# Libraries after end-group
PCCB_10_LDLIBS_POST := -Wl,--end-group -lm

# Object files
PCCB_10_CPP_OBJS := $(PCCB_10_CPP_SRCS:%.cpp=$(PCCB_10_BUILD_DIR)/%.o)
PCCB_10_CPP_OBJS := $(PCCB_10_CPP_OBJS:%.cc=$(PCCB_10_BUILD_DIR)/%.o)
PCCB_10_C_OBJS := $(PCCB_10_C_SRCS:%.c=$(PCCB_10_BUILD_DIR)/%.o)
PCCB_10_OBJS := $(PCCB_10_CPP_OBJS) $(PCCB_10_C_OBJS)

# Dependency files
PCCB_10_DEPS := $(PCCB_10_OBJS:.o=.d)

# Target rule
.PHONY: PCCB_10
PCCB_10: $(PCCB_10_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "PCCB_10 firmware build complete!"
	$(Q)echo "Output: $(PCCB_10_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(PCCB_10_TARGET_ELF)

# Link ELF file
$(PCCB_10_TARGET_ELF): $(PCCB_10_OBJS) $(PCCB_10_CANLIB_LIB) $(PCCB_10_COREN2G_LIB) $(PCCB_10_RRFLIBS_LIB) $(PCCB_10_FREERTOS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(PCCB_10_LDFLAGS1) -o $@ $(PCCB_10_LDFLAGS2) $(PCCB_10_OBJS) $(PCCB_10_LDLIBS) $(PCCB_10_LDLIBS_POST)

# Generate binary file
$(PCCB_10_TARGET_BIN): $(PCCB_10_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)if command -v CrcAppender >/dev/null 2>&1; then \
		echo "  CRC     $@"; \
		CrcAppender $@ PccbEmbeddedFiles; \
	fi

# Compile C++ files
$(PCCB_10_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(PCCB_10_CXXFLAGS) -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(PCCB_10_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(PCCB_10_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(PCCB_10_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(PCCB_10_CFLAGS) -Dnoexcept= -MMD -MP -o $@ $<

# Touch Version.cpp before build (pre-build step)
.PHONY: pccb_10-prebuild
pccb_10-prebuild:
	$(Q)touch -c $(PCCB_10_SRC_DIR)/Version.cpp 2>/dev/null || true

$(PCCB_10_OBJS): | pccb_10-prebuild

# Include dependencies
-include $(PCCB_10_DEPS)

# Clean target
.PHONY: clean-PCCB_10
clean-PCCB_10:
	$(Q)echo "  RM      $(PCCB_10_BUILD_DIR)"
	$(Q)rm -rf $(PCCB_10_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: pccb_10-libs
pccb_10-libs: $(PCCB_10_FREERTOS_LIB) $(PCCB_10_COREN2G_LIB) $(PCCB_10_RRFLIBS_LIB) $(PCCB_10_CANLIB_LIB)
	$(Q)echo "All required libraries built successfully"
