# RepRapFirmware Duet2_SBC Configuration Makefile
# Build configuration for Duet 2 with SBC (Single Board Computer) support

# Build directory and output
DUET2SBC_BUILD_DIR := Duet2_SBC
DUET2SBC_TARGET_NAME := Duet2Firmware_SBC
DUET2SBC_TARGET_ELF := $(DUET2SBC_BUILD_DIR)/$(DUET2SBC_TARGET_NAME).elf
DUET2SBC_TARGET_BIN := $(DUET2SBC_BUILD_DIR)/$(DUET2SBC_TARGET_NAME).bin
DUET2SBC_TARGET_MAP := $(DUET2SBC_BUILD_DIR)/$(DUET2SBC_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
DUET2SBC_FREERTOS_LIB := $(WORKSPACE)/FreeRTOS/SAM4E/libFreeRTOS.a
DUET2SBC_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAM4E_SDHC_USB_RTOS/libCoreN2G.a
DUET2SBC_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAM4E_RTOS/libRRFLibraries.a
DUET2SBC_CANLIB_LIB := $(WORKSPACE)/CANlib/SAM4E_RTOS/libCANlib.a

# Source directories
DUET2SBC_SRC_DIR := src

# Find libcpp and libc files first (must be linked first for proper malloc resolution)
DUET2SBC_LIBCPP_SRCS := $(shell find $(DUET2SBC_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
DUET2SBC_LIBC_SRCS := $(shell find $(DUET2SBC_SRC_DIR)/libc -name '*.c' -o -name '*.cpp' 2>/dev/null)

# Find all source files (excluding specified directories)
DUET2SBC_CPP_SRCS := $(shell find $(DUET2SBC_SRC_DIR) -name '*.cpp' \
	! -path '*/libcpp/*' \
	! -path '*/libc/*' \
	! -path '*/Networking/MQTT/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Hardware/ksz8081rna/*' \
	! -path '*/Networking/ESP8266WiFi/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Pccb/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/Networking/W5500Ethernet/*')

DUET2SBC_C_SRCS := $(shell find $(DUET2SBC_SRC_DIR) -name '*.c' \
	! -path '*/libc/*' \
	! -path '*/Networking/MQTT/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Hardware/ksz8081rna/*' \
	! -path '*/Networking/ESP8266WiFi/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Networking/LwipEthernet/*' \
	! -path '*/Pccb/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/Networking/W5500Ethernet/*')

# Include paths
DUET2SBC_INCLUDES := \
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
	-I$(DUET2SBC_SRC_DIR) \
	-I$(DUET2SBC_SRC_DIR)/Hardware/SAM4E \
	-I$(DUET2SBC_SRC_DIR)/DuetNG \
	-I$(DUET2SBC_SRC_DIR)/Networking

# Preprocessor defines
DUET2SBC_DEFINES := \
	-D__SAM4E8E__ \
	-DRTOS \
	-DDUET_NG \
	-DUSE_SBC \
	-D_XOPEN_SOURCE

# Compiler flags - C
DUET2SBC_CFLAGS := -c -std=gnu99 \
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
	-O2 \
	-Werror \
	-Wwrite-strings \
	$(DUET2SBC_INCLUDES) \
	$(DUET2SBC_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
DUET2SBC_CXXFLAGS := -c -std=gnu++17 \
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
	$(DUET2SBC_INCLUDES) \
	$(DUET2SBC_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - split into LDFLAGS1 (before -o) and LDFLAGS2 (after -o)
DUET2SBC_LDFLAGS1 := --specs=nosys.specs \
	-O2 \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(DUET2SBC_SRC_DIR)/Hardware/SAM4E/sam4e8e_flash.ld \
	-Wl,-Map,$(DUET2SBC_TARGET_MAP) \
	-mthumb

DUET2SBC_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths
DUET2SBC_LDLIBS := \
	-L$(WORKSPACE)/CANlib/SAM4E_RTOS \
	-L$(WORKSPACE)/CoreN2G/SAM4E_SDHC_USB_RTOS \
	-L$(WORKSPACE)/RRFLibraries/SAM4E_RTOS \
	-L$(WORKSPACE)/FreeRTOS/SAM4E \
	-lCANlib \
	-lCoreN2G \
	-lRRFLibraries \
	-lFreeRTOS \
	-lsupc++

DUET2SBC_LDLIBS_POST := -Wl,--end-group -lm

# Object files - libcpp and libc first for proper symbol resolution
DUET2SBC_LIBCPP_OBJS := $(DUET2SBC_LIBCPP_SRCS:%.cpp=$(DUET2SBC_BUILD_DIR)/%.o)
DUET2SBC_LIBCPP_OBJS := $(DUET2SBC_LIBCPP_OBJS:%.cc=$(DUET2SBC_BUILD_DIR)/%.o)
DUET2SBC_LIBC_OBJS := $(DUET2SBC_LIBC_SRCS:%.c=$(DUET2SBC_BUILD_DIR)/%.o)
DUET2SBC_LIBC_OBJS := $(DUET2SBC_LIBC_OBJS:%.cpp=$(DUET2SBC_BUILD_DIR)/%.o)
DUET2SBC_CPP_OBJS := $(DUET2SBC_CPP_SRCS:%.cpp=$(DUET2SBC_BUILD_DIR)/%.o)
DUET2SBC_CPP_OBJS := $(DUET2SBC_CPP_OBJS:%.cc=$(DUET2SBC_BUILD_DIR)/%.o)
DUET2SBC_C_OBJS := $(DUET2SBC_C_SRCS:%.c=$(DUET2SBC_BUILD_DIR)/%.o)
DUET2SBC_OBJS := $(DUET2SBC_LIBCPP_OBJS) $(DUET2SBC_LIBC_OBJS) $(DUET2SBC_CPP_OBJS) $(DUET2SBC_C_OBJS)

# Dependency files
DUET2SBC_DEPS := $(DUET2SBC_OBJS:.o=.d)

# Target rule
.PHONY: Duet2_SBC
Duet2_SBC: $(DUET2SBC_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet2_SBC firmware build complete!"
	$(Q)echo "Output: $(DUET2SBC_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(DUET2SBC_TARGET_ELF)

# Link ELF file
$(DUET2SBC_TARGET_ELF): $(DUET2SBC_OBJS) $(DUET2SBC_CANLIB_LIB) $(DUET2SBC_COREN2G_LIB) $(DUET2SBC_RRFLIBS_LIB) $(DUET2SBC_FREERTOS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(DUET2SBC_LDFLAGS1) -o $@ $(DUET2SBC_LDFLAGS2) -Wl,--start-group $(DUET2SBC_OBJS) $(DUET2SBC_LDLIBS) $(DUET2SBC_LDLIBS_POST)

# Generate binary file
$(DUET2SBC_TARGET_BIN): $(DUET2SBC_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)if command -v CrcAppender >/dev/null 2>&1; then \
		echo "  CRC     $@"; \
		CrcAppender $@; \
	fi

# Compile C++ files
$(DUET2SBC_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET2SBC_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(DUET2SBC_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(DUET2SBC_CFLAGS) -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(DUET2SBC_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET2SBC_CXXFLAGS) -MMD -MP -o $@ $<

# Touch Version.cpp before build (pre-build step)
.PHONY: duet2sbc-prebuild
duet2sbc-prebuild:
	$(Q)touch -c $(DUET2SBC_SRC_DIR)/Version.cpp 2>/dev/null || true

$(DUET2SBC_OBJS): | duet2sbc-prebuild

# Include dependencies
-include $(DUET2SBC_DEPS)

# Clean target
.PHONY: clean-Duet2_SBC
clean-Duet2_SBC:
	$(Q)echo "  RM      $(DUET2SBC_BUILD_DIR)"
	$(Q)rm -rf $(DUET2SBC_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: duet2sbc-libs
duet2sbc-libs: $(DUET2SBC_FREERTOS_LIB) $(DUET2SBC_COREN2G_LIB) $(DUET2SBC_RRFLIBS_LIB) $(DUET2SBC_CANLIB_LIB)
	$(Q)echo "All required libraries built successfully"
