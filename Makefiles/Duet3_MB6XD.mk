# RepRapFirmware Duet3_MB6XD Configuration Makefile
# Build configuration for Duet 3 MB6XD expansion board (SAME70 MCU)

# Build directory and output
DUET3MB6XD_BUILD_DIR := Duet3_MB6XD
DUET3MB6XD_TARGET_NAME := Duet3Firmware_MB6XD
DUET3MB6XD_TARGET_ELF := $(DUET3MB6XD_BUILD_DIR)/$(DUET3MB6XD_TARGET_NAME).elf
DUET3MB6XD_TARGET_BIN := $(DUET3MB6XD_BUILD_DIR)/$(DUET3MB6XD_TARGET_NAME).bin
DUET3MB6XD_TARGET_MAP := $(DUET3MB6XD_BUILD_DIR)/$(DUET3MB6XD_TARGET_NAME).map

# External library root
LIBRARIES_DIR ?= libraries

# Library dependencies
DUET3MB6XD_FREERTOS_LIB := $(LIBRARIES_DIR)/FreeRTOS/SAME70/libFreeRTOS.a
DUET3MB6XD_COREN2G_LIB := $(LIBRARIES_DIR)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS/libCoreN2G.a
DUET3MB6XD_RRFLIBS_LIB := $(LIBRARIES_DIR)/RRFLibraries/SAME70_RTOS/libRRFLibraries.a
DUET3MB6XD_CANLIB_LIB := $(LIBRARIES_DIR)/CANlib/SAME70_RTOS/libCANlib.a
DUET3MB6XD_LIBTINYUSB_LIB := $(LIBRARIES_DIR)/LibTinyusb/SAME70/libLibTinyusb.a
DUET3MB6XD_MBEDTLS_LIB := $(LIBRARIES_DIR)/LibMbedTls/SAME70/libLibMbedTls.a

# Source directories
DUET3MB6XD_SRC_DIR := src

# Find libcpp and libc files first (must be linked first for proper malloc resolution)
DUET3MB6XD_LIBCPP_SRCS := $(shell find $(DUET3MB6XD_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
DUET3MB6XD_LIBC_SRCS := $(shell find $(DUET3MB6XD_SRC_DIR)/libc -name '*.c' -o -name '*.cpp' 2>/dev/null)

# Find all source files (excluding specified directories)
DUET3MB6XD_CPP_SRCS := $(shell find $(DUET3MB6XD_SRC_DIR) -name '*.cpp' \
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

DUET3MB6XD_C_SRCS := $(shell find $(DUET3MB6XD_SRC_DIR) -name '*.c' \
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
DUET3MB6XD_INCLUDES := \
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
	-I$(DUET3MB6XD_SRC_DIR) \
	-I$(DUET3MB6XD_SRC_DIR)/Hardware/SAME70 \
	-I$(DUET3MB6XD_SRC_DIR)/Networking \
	-I$(DUET3MB6XD_SRC_DIR)/Networking/LwipEthernet/Lwip \
	-I$(DUET3MB6XD_SRC_DIR)/Networking/LwipEthernet/Lwip/src/include \
	-I$(DUET3MB6XD_SRC_DIR)/Networking/MQTT/MQTT_C/include

# Preprocessor defines
MQTTC_PAL_DEFINE := Networking/MQTT/mqtt_pal.h
DUET3MB6XD_DEFINES := \
	-D__SAME70Q20B__ \
	-DRTOS \
	-DDUET3_MB6XD \
	-DMBEDTLS_CONFIG_FILE='"config-same70.h"'

# Compiler flags - C
DUET3MB6XD_CFLAGS := -c -std=gnu99 \
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
	$(DUET3MB6XD_INCLUDES) \
	$(DUET3MB6XD_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
DUET3MB6XD_CXXFLAGS := -c -std=c++20 \
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
	$(DUET3MB6XD_INCLUDES) \
	$(DUET3MB6XD_DEFINES) \
	-D_XOPEN_SOURCE

# Linker flags - split into LDFLAGS1 (before -o) and LDFLAGS2 (after -o)
DUET3MB6XD_LDFLAGS1 := --specs=nosys.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m7 \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-T$(DUET3MB6XD_SRC_DIR)/Hardware/SAME70/same70q20b_flash.ld \
	-Wl,-Map,$(DUET3MB6XD_TARGET_MAP) \
	-mthumb

DUET3MB6XD_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths
DUET3MB6XD_LDLIBS := \
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

DUET3MB6XD_LDLIBS_POST := -Wl,--end-group -lm

# Object files - libcpp and libc first for proper symbol resolution
DUET3MB6XD_LIBCPP_OBJS := $(DUET3MB6XD_LIBCPP_SRCS:%.cpp=$(DUET3MB6XD_BUILD_DIR)/%.o)
DUET3MB6XD_LIBCPP_OBJS := $(DUET3MB6XD_LIBCPP_OBJS:%.cc=$(DUET3MB6XD_BUILD_DIR)/%.o)
DUET3MB6XD_LIBC_OBJS := $(DUET3MB6XD_LIBC_SRCS:%.c=$(DUET3MB6XD_BUILD_DIR)/%.o)
DUET3MB6XD_LIBC_OBJS := $(DUET3MB6XD_LIBC_OBJS:%.cpp=$(DUET3MB6XD_BUILD_DIR)/%.o)
DUET3MB6XD_CPP_OBJS := $(DUET3MB6XD_CPP_SRCS:%.cpp=$(DUET3MB6XD_BUILD_DIR)/%.o)
DUET3MB6XD_CPP_OBJS := $(DUET3MB6XD_CPP_OBJS:%.cc=$(DUET3MB6XD_BUILD_DIR)/%.o)
DUET3MB6XD_C_OBJS := $(DUET3MB6XD_C_SRCS:%.c=$(DUET3MB6XD_BUILD_DIR)/%.o)
DUET3MB6XD_OBJS := $(DUET3MB6XD_LIBCPP_OBJS) $(DUET3MB6XD_LIBC_OBJS) $(DUET3MB6XD_CPP_OBJS) $(DUET3MB6XD_C_OBJS)

# Dependency files
DUET3MB6XD_DEPS := $(DUET3MB6XD_OBJS:.o=.d)

# Target rule
.PHONY: Duet3_MB6XD
Duet3_MB6XD: $(DUET3MB6XD_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet3_MB6XD firmware build complete!"
	$(Q)echo "Output: $(DUET3MB6XD_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(DUET3MB6XD_TARGET_ELF)

# Link ELF file
$(DUET3MB6XD_TARGET_ELF): $(DUET3MB6XD_OBJS) $(DUET3MB6XD_CANLIB_LIB) $(DUET3MB6XD_COREN2G_LIB) $(DUET3MB6XD_RRFLIBS_LIB) $(DUET3MB6XD_FREERTOS_LIB) $(DUET3MB6XD_LIBTINYUSB_LIB) $(DUET3MB6XD_MBEDTLS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(DUET3MB6XD_LDFLAGS1) -o $@ $(DUET3MB6XD_LDFLAGS2) -Wl,--start-group $(DUET3MB6XD_OBJS) $(DUET3MB6XD_LDLIBS) $(DUET3MB6XD_LDLIBS_POST)

# Generate binary file
$(DUET3MB6XD_TARGET_BIN): $(DUET3MB6XD_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)command -v CrcAppender >/dev/null 2>&1 || { echo "CrcAppender not found on PATH" >&2; exit 1; }
	$(Q)echo "  CRC     $@"
	$(Q)CrcAppender $@

# Compile C++ files
$(DUET3MB6XD_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3MB6XD_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile C files
$(DUET3MB6XD_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(DUET3MB6XD_CFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(DUET3MB6XD_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3MB6XD_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Touch Version.cpp before build (pre-build step)
.PHONY: duet3mb6xd-prebuild
duet3mb6xd-prebuild:
	$(Q)touch -c $(DUET3MB6XD_SRC_DIR)/Version.cpp 2>/dev/null || true

$(DUET3MB6XD_OBJS): | duet3mb6xd-prebuild

# Include dependencies
-include $(DUET3MB6XD_DEPS)

# Clean target
.PHONY: clean-Duet3_MB6XD
clean-Duet3_MB6XD:
	$(Q)echo "  RM      $(DUET3MB6XD_BUILD_DIR)"
	$(Q)rm -rf $(DUET3MB6XD_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: duet3mb6xd-libs
duet3mb6xd-libs: $(DUET3MB6XD_FREERTOS_LIB) $(DUET3MB6XD_COREN2G_LIB) $(DUET3MB6XD_RRFLIBS_LIB) $(DUET3MB6XD_CANLIB_LIB) $(DUET3MB6XD_LIBTINYUSB_LIB) $(DUET3MB6XD_MBEDTLS_LIB)
	$(Q)echo "All required libraries built successfully"
