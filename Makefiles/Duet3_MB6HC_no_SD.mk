# RepRapFirmware Duet3_MB6HC_no_SD Configuration Makefile
# Build configuration for Duet 3 Main Board 6HC (SAME70 MCU)

# Build directory and output
DUET3NOSD_BUILD_DIR := Duet3_MB6HC_no_SD
DUET3NOSD_TARGET_NAME := Duet3Firmware_MB6HC
DUET3NOSD_TARGET_ELF := $(DUET3NOSD_BUILD_DIR)/$(DUET3NOSD_TARGET_NAME).elf
DUET3NOSD_TARGET_BIN := $(DUET3NOSD_BUILD_DIR)/$(DUET3NOSD_TARGET_NAME).bin
DUET3NOSD_TARGET_MAP := $(DUET3NOSD_BUILD_DIR)/$(DUET3NOSD_TARGET_NAME).map

# External library root
LIBRARIES_DIR ?= libraries

# Library dependencies
DUET3NOSD_FREERTOS_LIB := $(LIBRARIES_DIR)/FreeRTOS/SAME70/libFreeRTOS.a
DUET3NOSD_COREN2G_LIB := $(LIBRARIES_DIR)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS/libCoreN2G.a
DUET3NOSD_RRFLIBS_LIB := $(LIBRARIES_DIR)/RRFLibraries/SAME70_RTOS/libRRFLibraries.a
DUET3NOSD_CANLIB_LIB := $(LIBRARIES_DIR)/CANlib/SAME70_RTOS/libCANlib.a
DUET3NOSD_LIBTINYUSB_LIB := $(LIBRARIES_DIR)/LibTinyusb/SAME70/libLibTinyusb.a
DUET3NOSD_MBEDTLS_LIB := $(LIBRARIES_DIR)/LibMbedTls/SAME70/libLibMbedTls.a

# Source directories
DUET3NOSD_SRC_DIR := src

# Find libcpp and libc files first (must be linked first for proper malloc resolution)
DUET3NOSD_LIBCPP_SRCS := $(shell find $(DUET3NOSD_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
DUET3NOSD_LIBC_SRCS := $(shell find $(DUET3NOSD_SRC_DIR)/libc -name '*.c' -o -name '*.cpp' 2>/dev/null)

# Find all source files (excluding specified directories)
DUET3NOSD_CPP_SRCS := $(shell find $(DUET3NOSD_SRC_DIR) -name '*.cpp' \
	! -path '*/libcpp/*' \
	! -path '*/libc/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/smtp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/snmp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/httpd/*' \
	! -path '*/Networking/LwipEthernet/Lwip/test/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/tftp/*' \
	! -path '*/Networking/W5500Ethernet/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/netif/ppp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/lwiperf/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/sntp/*' \
	! -path '*/Display/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/http/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Pccb/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/mqtt/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/Networking/LwipEthernet/Lwip/doc/*')

DUET3NOSD_C_SRCS := $(shell find $(DUET3NOSD_SRC_DIR) -name '*.c' \
	! -path '*/libc/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/smtp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/snmp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/test/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/tftp/*' \
	! -path '*/Networking/W5500Ethernet/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/netif/ppp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/lwiperf/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/sntp/*' \
	! -path '*/Display/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/http/*' \
	! -path '*/Duet3Mini/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/Hardware/SAME5x/*' \
	! -path '*/Pccb/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/mqtt/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetM/*' \
	! -path '*/Networking/LwipEthernet/Lwip/doc/*' \
	! -path '*/MQTT_C/tests.c' \
	! -path '*/MQTT_C/examples/*' \
	! -path '*/MQTT_C/src/mqtt_pal.c')

# Include paths
DUET3NOSD_INCLUDES := \
	-I$(LIBRARIES_DIR)/LibMbedTls/include \
	-I$(LIBRARIES_DIR)/LibMbedTls/library \
	-I$(LIBRARIES_DIR)/LibMbedTls/configs \
	-I$(LIBRARIES_DIR)/LibTinyusb \
	-I$(LIBRARIES_DIR)/CoreN2G \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAM4S_4E_E70/SAME70 \
	-I$(LIBRARIES_DIR)/FreeRTOS \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/include \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/portable/GCC/ARM_CM7/r0p1 \
	-I$(LIBRARIES_DIR)/RRFLibraries/src \
	-I$(LIBRARIES_DIR)/CANlib/src \
	-I$(LIBRARIES_DIR)/WiFiSocketServerRTOS/src/include \
	-I$(DUET3NOSD_SRC_DIR) \
	-I$(DUET3NOSD_SRC_DIR)/Hardware/SAME70 \
	-I$(DUET3NOSD_SRC_DIR)/Networking \
	-I$(DUET3NOSD_SRC_DIR)/Networking/LwipEthernet/Lwip \
	-I$(DUET3NOSD_SRC_DIR)/Networking/LwipEthernet/Lwip/src/include \
	-I$(DUET3NOSD_SRC_DIR)/Networking/MQTT/MQTT_C/include

# Preprocessor defines
MQTTC_PAL_DEFINE := Networking/MQTT/mqtt_pal.h
DUET3NOSD_DEFINES := \
	-D__SAME70Q20B__ \
	-DRTOS \
	-DDUET3_MB6HC \
	-DMBEDTLS_CONFIG_FILE='"config-same70.h"'

# Compiler flags - C
DUET3NOSD_CFLAGS := -c -std=gnu99 \
	-Wall \
	-mcpu=cortex-m7 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-mno-unaligned-access \
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
	$(DUET3NOSD_INCLUDES) \
	$(DUET3NOSD_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
DUET3NOSD_CXXFLAGS := -c -std=c++20 \
	-Wall \
	-mcpu=cortex-m7 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-mno-unaligned-access \
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
	$(DUET3NOSD_INCLUDES) \
	$(DUET3NOSD_DEFINES) \
	-D_XOPEN_SOURCE \
	$(DEBUG_FLAGS)

# Linker flags - split into LDFLAGS1 (before -o) and LDFLAGS2 (after -o)
DUET3NOSD_LDFLAGS1 := --specs=nosys.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m7 \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-T$(DUET3NOSD_SRC_DIR)/Hardware/SAME70/same70q20b_flash.ld \
	-Wl,-Map,$(DUET3NOSD_TARGET_MAP) \
	-mthumb

DUET3NOSD_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths
DUET3NOSD_LDLIBS := \
	-L$(LIBRARIES_DIR)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS \
	-L$(LIBRARIES_DIR)/RRFLibraries/SAME70_RTOS \
	-L$(LIBRARIES_DIR)/FreeRTOS/SAME70 \
	-L$(LIBRARIES_DIR)/CANlib/SAME70_RTOS \
	-L$(LIBRARIES_DIR)/LibTinyusb/SAME70 \
	-L$(LIBRARIES_DIR)/LibMbedTls/SAME70 \
	-lLibTinyusb \
	-lLibMbedTls \
	-lCoreN2G \
	-lRRFLibraries \
	-lFreeRTOS \
	-lCANlib \
	-lsupc++

DUET3NOSD_LDLIBS_POST := -Wl,--end-group -lm

# Object files - libcpp and libc first for proper symbol resolution
DUET3NOSD_LIBCPP_OBJS := $(DUET3NOSD_LIBCPP_SRCS:%.cpp=$(DUET3NOSD_BUILD_DIR)/%.o)
DUET3NOSD_LIBCPP_OBJS := $(DUET3NOSD_LIBCPP_OBJS:%.cc=$(DUET3NOSD_BUILD_DIR)/%.o)
DUET3NOSD_LIBC_OBJS := $(DUET3NOSD_LIBC_SRCS:%.c=$(DUET3NOSD_BUILD_DIR)/%.o)
DUET3NOSD_LIBC_OBJS := $(DUET3NOSD_LIBC_OBJS:%.cpp=$(DUET3NOSD_BUILD_DIR)/%.o)
DUET3NOSD_CPP_OBJS := $(DUET3NOSD_CPP_SRCS:%.cpp=$(DUET3NOSD_BUILD_DIR)/%.o)
DUET3NOSD_CPP_OBJS := $(DUET3NOSD_CPP_OBJS:%.cc=$(DUET3NOSD_BUILD_DIR)/%.o)
DUET3NOSD_C_OBJS := $(DUET3NOSD_C_SRCS:%.c=$(DUET3NOSD_BUILD_DIR)/%.o)
DUET3NOSD_OBJS := $(DUET3NOSD_LIBCPP_OBJS) $(DUET3NOSD_LIBC_OBJS) $(DUET3NOSD_CPP_OBJS) $(DUET3NOSD_C_OBJS)

# Dependency files
DUET3NOSD_DEPS := $(DUET3NOSD_OBJS:.o=.d)

# Target rule
.PHONY: Duet3_MB6HC_no_SD
Duet3_MB6HC_no_SD: $(DUET3NOSD_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet3_MB6HC_no_SD firmware build complete!"
	$(Q)echo "Output: $(DUET3NOSD_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(DUET3NOSD_TARGET_ELF)

# Link ELF file
$(DUET3NOSD_TARGET_ELF): $(DUET3NOSD_OBJS) $(DUET3NOSD_CANLIB_LIB) $(DUET3NOSD_COREN2G_LIB) $(DUET3NOSD_RRFLIBS_LIB) $(DUET3NOSD_FREERTOS_LIB) $(DUET3NOSD_LIBTINYUSB_LIB) $(DUET3NOSD_MBEDTLS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(DUET3NOSD_LDFLAGS1) -o $@ $(DUET3NOSD_LDFLAGS2) -Wl,--start-group $(DUET3NOSD_OBJS) $(DUET3NOSD_LDLIBS) $(DUET3NOSD_LDLIBS_POST)

# Generate binary file
$(DUET3NOSD_TARGET_BIN): $(DUET3NOSD_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)command -v CrcAppender >/dev/null 2>&1 || { echo "CrcAppender not found on PATH" >&2; exit 1; }
	$(Q)echo "  CRC     $@"
	$(Q)CrcAppender $@

# Compile C++ files
$(DUET3NOSD_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3NOSD_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile C files
$(DUET3NOSD_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(DUET3NOSD_CFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(DUET3NOSD_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3NOSD_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Touch Version.cpp before build (pre-build step)
.PHONY: duet3nosd-prebuild
duet3nosd-prebuild:
	$(Q)touch -c $(DUET3NOSD_SRC_DIR)/Version.cpp 2>/dev/null || true

$(DUET3NOSD_OBJS): | duet3nosd-prebuild

# Include dependencies
-include $(DUET3NOSD_DEPS)

# Clean target
.PHONY: clean-Duet3_MB6HC_no_SD
clean-Duet3_MB6HC_no_SD:
	$(Q)echo "  RM      $(DUET3NOSD_BUILD_DIR)"
	$(Q)rm -rf $(DUET3NOSD_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: duet3nosd-libs
duet3nosd-libs: $(DUET3NOSD_FREERTOS_LIB) $(DUET3NOSD_COREN2G_LIB) $(DUET3NOSD_RRFLIBS_LIB) $(DUET3NOSD_CANLIB_LIB) $(DUET3NOSD_LIBTINYUSB_LIB) $(DUET3NOSD_MBEDTLS_LIB)
	$(Q)echo "All required libraries built successfully"
