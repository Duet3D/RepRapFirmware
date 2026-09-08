# Duet 3 MB6HC built with USE_EMBEDDED_FILES.
#
# Derived from Duet3_MB6HC.mk. The only functional difference is -DUSE_EMBEDDED_FILES, which in
# Pins_Duet3_MB6HC.h turns off mass storage, high-speed SD and the SBC interface and turns on the
# embedded filesystem that lives after _firmware_end.
#
# This exists for the Renode emulator: modelling HSMCI is a large job, and without an SD card there
# is no config.g, so no M575, so the aux UART never opens and there is no way to send G-code in.
# Embedding the files removes the SD card from that chain entirely.
#
# RepRapFirmware Duet3_MB6HC Configuration Makefile
# Build configuration for Duet 3 Main Board 6HC (SAME70 MCU)

# Build directory and output
DUET3EMB_BUILD_DIR := Duet3_MB6HC_embedded
DUET3EMB_TARGET_NAME := Duet3Firmware_MB6HC_embedded
DUET3EMB_TARGET_ELF := $(DUET3EMB_BUILD_DIR)/$(DUET3EMB_TARGET_NAME).elf
DUET3EMB_TARGET_BIN := $(DUET3EMB_BUILD_DIR)/$(DUET3EMB_TARGET_NAME).bin
DUET3EMB_TARGET_MAP := $(DUET3EMB_BUILD_DIR)/$(DUET3EMB_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
DUET3EMB_FREERTOS_LIB := $(WORKSPACE)/FreeRTOS/SAME70/libFreeRTOS.a
DUET3EMB_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS/libCoreN2G.a
DUET3EMB_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAME70_RTOS/libRRFLibraries.a
DUET3EMB_CANLIB_LIB := $(WORKSPACE)/CANlib/SAME70_RTOS/libCANlib.a
DUET3EMB_LIBTINYUSB_LIB := $(WORKSPACE)/LibTinyusb/SAME70/libLibTinyusb.a
DUET3EMB_MBEDTLS_LIB := $(WORKSPACE)/LibMbedTls/SAME70/libLibMbedTls.a

# Source directories
DUET3EMB_SRC_DIR := src

# Find libcpp and libc files first (must be linked first for proper malloc resolution)
DUET3EMB_LIBCPP_SRCS := $(shell find $(DUET3EMB_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
DUET3EMB_LIBC_SRCS := $(shell find $(DUET3EMB_SRC_DIR)/libc -name '*.c' -o -name '*.cpp' 2>/dev/null)

# Find all source files (excluding specified directories)
DUET3EMB_CPP_SRCS := $(shell find $(DUET3EMB_SRC_DIR) -name '*.cpp' \
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

DUET3EMB_C_SRCS := $(shell find $(DUET3EMB_SRC_DIR) -name '*.c' \
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
DUET3EMB_INCLUDES := \
	-I$(WORKSPACE)/LibMbedTls/include \
	-I$(WORKSPACE)/LibMbedTls/library \
	-I$(WORKSPACE)/LibMbedTls/configs \
	-I$(WORKSPACE)/LibTinyusb \
	-I$(WORKSPACE)/CoreN2G \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAME70 \
	-I$(WORKSPACE)/FreeRTOS \
	-I$(WORKSPACE)/FreeRTOS/src/include \
	-I$(WORKSPACE)/FreeRTOS/src/portable/GCC/ARM_CM7/r0p1 \
	-I$(WORKSPACE)/RRFLibraries/src \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/WiFiSocketServerRTOS/src/include \
	-I$(DUET3EMB_SRC_DIR) \
	-I$(DUET3EMB_SRC_DIR)/Hardware/SAME70 \
	-I$(DUET3EMB_SRC_DIR)/Networking \
	-I$(DUET3EMB_SRC_DIR)/Networking/LwipEthernet/Lwip \
	-I$(DUET3EMB_SRC_DIR)/Networking/LwipEthernet/Lwip/src/include \
	-I$(DUET3EMB_SRC_DIR)/Networking/MQTT/MQTT_C/include

# Preprocessor defines
MQTTC_PAL_DEFINE := Networking/MQTT/mqtt_pal.h
DUET3EMB_DEFINES := \
	-D__SAME70Q20B__ \
	-DRTOS \
	-DDUET3_MB6HC \
	-DMBEDTLS_CONFIG_FILE='"config-same70.h"' \
	-DUSE_EMBEDDED_FILES

# Compiler flags - C
DUET3EMB_CFLAGS := -c -std=gnu99 \
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
	$(DUET3EMB_INCLUDES) \
	$(DUET3EMB_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
DUET3EMB_CXXFLAGS := -c -std=c++20 \
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
	$(DUET3EMB_INCLUDES) \
	$(DUET3EMB_DEFINES) \
	-D_XOPEN_SOURCE \
	$(DEBUG_FLAGS)

# Linker flags - split into LDFLAGS1 (before -o) and LDFLAGS2 (after -o)
DUET3EMB_LDFLAGS1 := --specs=nosys.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m7 \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-T$(DUET3EMB_SRC_DIR)/Hardware/SAME70/same70q20b_flash.ld \
	-Wl,-Map,$(DUET3EMB_TARGET_MAP) \
	-mthumb

DUET3EMB_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths
DUET3EMB_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME70_CAN_SDHC_USB_RTOS \
	-L$(WORKSPACE)/RRFLibraries/SAME70_RTOS \
	-L$(WORKSPACE)/FreeRTOS/SAME70 \
	-L$(WORKSPACE)/CANlib/SAME70_RTOS \
	-L$(WORKSPACE)/LibTinyusb/SAME70 \
	-L$(WORKSPACE)/LibMbedTls/SAME70 \
	-lLibTinyusb \
	-lLibMbedTls \
	-lCoreN2G \
	-lRRFLibraries \
	-lFreeRTOS \
	-lCANlib \
	-lsupc++

DUET3EMB_LDLIBS_POST := -Wl,--end-group -lm

# Object files - libcpp and libc first for proper symbol resolution
DUET3EMB_LIBCPP_OBJS := $(DUET3EMB_LIBCPP_SRCS:%.cpp=$(DUET3EMB_BUILD_DIR)/%.o)
DUET3EMB_LIBCPP_OBJS := $(DUET3EMB_LIBCPP_OBJS:%.cc=$(DUET3EMB_BUILD_DIR)/%.o)
DUET3EMB_LIBC_OBJS := $(DUET3EMB_LIBC_SRCS:%.c=$(DUET3EMB_BUILD_DIR)/%.o)
DUET3EMB_LIBC_OBJS := $(DUET3EMB_LIBC_OBJS:%.cpp=$(DUET3EMB_BUILD_DIR)/%.o)
DUET3EMB_CPP_OBJS := $(DUET3EMB_CPP_SRCS:%.cpp=$(DUET3EMB_BUILD_DIR)/%.o)
DUET3EMB_CPP_OBJS := $(DUET3EMB_CPP_OBJS:%.cc=$(DUET3EMB_BUILD_DIR)/%.o)
DUET3EMB_C_OBJS := $(DUET3EMB_C_SRCS:%.c=$(DUET3EMB_BUILD_DIR)/%.o)
DUET3EMB_OBJS := $(DUET3EMB_LIBCPP_OBJS) $(DUET3EMB_LIBC_OBJS) $(DUET3EMB_CPP_OBJS) $(DUET3EMB_C_OBJS)

# Dependency files
DUET3EMB_DEPS := $(DUET3EMB_OBJS:.o=.d)

# Target rule
.PHONY: Duet3_MB6HC_embedded
Duet3_MB6HC_embedded: $(DUET3EMB_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet3_MB6HC firmware build complete!"
	$(Q)echo "Output: $(DUET3EMB_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(DUET3EMB_TARGET_ELF)

# Link ELF file
$(DUET3EMB_TARGET_ELF): $(DUET3EMB_OBJS) $(DUET3EMB_CANLIB_LIB) $(DUET3EMB_COREN2G_LIB) $(DUET3EMB_RRFLIBS_LIB) $(DUET3EMB_FREERTOS_LIB) $(DUET3EMB_LIBTINYUSB_LIB) $(DUET3EMB_MBEDTLS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(DUET3EMB_LDFLAGS1) -o $@ $(DUET3EMB_LDFLAGS2) -Wl,--start-group $(DUET3EMB_OBJS) $(DUET3EMB_LDLIBS) $(DUET3EMB_LDLIBS_POST)

# Generate binary file
$(DUET3EMB_TARGET_BIN): $(DUET3EMB_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)if command -v CrcAppender >/dev/null 2>&1; then \
		echo "  CRC     $@"; \
		CrcAppender $@; \
	fi

# Compile C++ files
$(DUET3EMB_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3EMB_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile C files
$(DUET3EMB_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(DUET3EMB_CFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(DUET3EMB_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3EMB_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Touch Version.cpp before build (pre-build step)
.PHONY: duet3mb6hc-embedded-prebuild
duet3mb6hc-embedded-prebuild:
	$(Q)touch -c $(DUET3EMB_SRC_DIR)/Version.cpp 2>/dev/null || true

$(DUET3EMB_OBJS): | duet3mb6hc-embedded-prebuild

# Include dependencies
-include $(DUET3EMB_DEPS)

# Clean target
.PHONY: clean-Duet3_MB6HC_embedded
clean-Duet3_MB6HC_embedded:
	$(Q)echo "  RM      $(DUET3EMB_BUILD_DIR)"
	$(Q)rm -rf $(DUET3EMB_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: duet3mb6hc-embedded-libs
duet3mb6hc-embedded-libs: $(DUET3EMB_FREERTOS_LIB) $(DUET3EMB_COREN2G_LIB) $(DUET3EMB_RRFLIBS_LIB) $(DUET3EMB_CANLIB_LIB) $(DUET3EMB_LIBTINYUSB_LIB) $(DUET3EMB_MBEDTLS_LIB)
	$(Q)echo "All required libraries built successfully"
