# RepRapFirmware Duet3Mini5plus Configuration Makefile
# Build configuration for Duet 3 Mini 5+ boards (SAME51 MCU)

DUET3MINI_BUILD_DIR := Duet3Mini5plus
DUET3MINI_TARGET_NAME := Duet3Firmware_Mini5plus
DUET3MINI_TARGET_ELF := $(DUET3MINI_BUILD_DIR)/$(DUET3MINI_TARGET_NAME).elf
DUET3MINI_TARGET_BIN := $(DUET3MINI_BUILD_DIR)/$(DUET3MINI_TARGET_NAME).bin
DUET3MINI_TARGET_UF2 := $(DUET3MINI_BUILD_DIR)/$(DUET3MINI_TARGET_NAME).uf2
DUET3MINI_TARGET_MAP := $(DUET3MINI_BUILD_DIR)/$(DUET3MINI_TARGET_NAME).map

LIBRARIES_DIR ?= libraries

# Library dependencies
DUET3MINI_FREERTOS_LIB := $(LIBRARIES_DIR)/FreeRTOS/SAME51/libFreeRTOS.a
DUET3MINI_COREN2G_LIB := $(LIBRARIES_DIR)/CoreN2G/SAME5x_CAN_SDHC_USB_RTOS/libCoreN2G.a
DUET3MINI_RRFLIBS_LIB := $(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a
DUET3MINI_CANLIB_LIB := $(LIBRARIES_DIR)/CANlib/SAME51_RTOS/libCANlib.a
DUET3MINI_LIBTINYUSB_LIB := $(LIBRARIES_DIR)/LibTinyusb/SAME5x/libLibTinyusb.a
DUET3MINI_MBEDTLS_LIB := $(LIBRARIES_DIR)/LibMbedTls/SAME5x/libLibMbedTls.a

DUET3MINI_SRC_DIR := src

# Find libcpp and libc files first (must be linked first for proper malloc resolution)
DUET3MINI_LIBCPP_SRCS := $(shell find $(DUET3MINI_SRC_DIR)/libcpp -name '*.cpp' -o -name '*.cc' 2>/dev/null)
DUET3MINI_LIBC_SRCS := $(shell find $(DUET3MINI_SRC_DIR)/libc -name '*.c' -o -name '*.cpp' 2>/dev/null)

# Find all source files (excluding specified directories)
DUET3MINI_CPP_SRCS := $(shell find $(DUET3MINI_SRC_DIR) -name '*.cpp' \
	! -path '*/libcpp/*' \
	! -path '*/libc/*' \
	! -path '*/Duet3_V06/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Networking/W5500Ethernet/*' \
	! -path '*/Pccb/*' \
	! -path '*/DuetM/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/smtp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/snmp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/tftp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/lwiperf/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/sntp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/http/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/mqtt/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/netif/ppp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/doc/*')

DUET3MINI_C_SRCS := $(shell find $(DUET3MINI_SRC_DIR) -name '*.c' \
	! -path '*/libc/*' \
	! -path '*/SBC/*' \
	! -path '*/Hardware/SAME70/*' \
	! -path '*/Hardware/SAM4E/*' \
	! -path '*/Hardware/SAM4S/*' \
	! -path '*/DuetNG/*' \
	! -path '*/Pccb/*' \
	! -path '*/DuetM/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/smtp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/snmp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/tftp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/lwiperf/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/sntp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/http/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/apps/mqtt/*' \
	! -path '*/Networking/LwipEthernet/Lwip/src/netif/ppp/*' \
	! -path '*/Networking/LwipEthernet/Lwip/test/*' \
	! -path '*/Networking/LwipEthernet/Lwip/doc/*' \
	! -path '*/MQTT_C/tests.c' \
	! -path '*/MQTT_C/examples/*' \
	! -path '*/MQTT_C/src/mqtt_pal.c')

# Include paths
DUET3MINI_INCLUDES := \
	-I$(LIBRARIES_DIR)/LibMbedTls/include \
	-I$(LIBRARIES_DIR)/LibMbedTls/library \
	-I$(LIBRARIES_DIR)/LibMbedTls/configs \
	-I$(LIBRARIES_DIR)/CoreN2G \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21 \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hal/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hri \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/Config \
	-I$(LIBRARIES_DIR)/CoreN2G/src/atmel/SAME54_DFP/1.1.134/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(LIBRARIES_DIR)/FreeRTOS \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/include \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/portable/GCC/ARM_CM4F \
	-I$(LIBRARIES_DIR)/RRFLibraries/src \
	-I$(LIBRARIES_DIR)/LibTinyusb/src \
	-I$(LIBRARIES_DIR)/CANlib/src \
	-I$(LIBRARIES_DIR)/WiFiSocketServerRTOS/src/include \
	-I$(DUET3MINI_SRC_DIR) \
	-I$(DUET3MINI_SRC_DIR)/Hardware/SAME5x \
	-I$(DUET3MINI_SRC_DIR)/Duet3Mini \
	-I$(DUET3MINI_SRC_DIR)/Networking \
	-I$(DUET3MINI_SRC_DIR)/Networking/LwipEthernet/Lwip \
	-I$(DUET3MINI_SRC_DIR)/Networking/LwipEthernet/Lwip/src/include \
	-I$(DUET3MINI_SRC_DIR)/Networking/MQTT/MQTT_C/include \
	-I$(DUET3MINI_SRC_DIR)/Hardware/SAME5x/Ethernet

# Preprocessor defines
# Note: MQTTC_PAL_FILE needs special handling for quotes
MQTTC_PAL_DEFINE := Networking/MQTT/mqtt_pal.h
DUET3MINI_DEFINES := \
	-D__SAME54P20A__ \
	-DRTOS \
	-DDUET3MINI_V04 \
	-DMBEDTLS_CONFIG_FILE='"config-same5x.h"'

# Compiler flags - C
DUET3MINI_CFLAGS := -c -std=gnu99 \
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
	-O2 \
	-Werror \
	-Wwrite-strings \
	$(DUET3MINI_INCLUDES) \
	$(DUET3MINI_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
DUET3MINI_CXXFLAGS := -c -std=c++20 \
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
	-O2 \
	-Werror \
	-Wnoexcept \
	-Wshadow \
	-Wsign-promo \
	$(DUET3MINI_INCLUDES) \
	$(DUET3MINI_DEFINES) \
	-D_XOPEN_SOURCE \
	$(DEBUG_FLAGS)

# Linker flags - split into LDFLAGS1 (before -o) and LDFLAGS2 (after -o)
DUET3MINI_LDFLAGS1 := --specs=nosys.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-Wl,--no-warn-rwx-segment \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mthumb \
	-T$(DUET3MINI_SRC_DIR)/Hardware/SAME5x/same54p20a_flash_16k_bootloader.ld \
	-Wl,-Map,$(DUET3MINI_TARGET_MAP)

DUET3MINI_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths
DUET3MINI_LDLIBS := \
	-L$(LIBRARIES_DIR)/CoreN2G/SAME5x_CAN_SDHC_USB_RTOS \
	-L$(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS \
	-L$(LIBRARIES_DIR)/FreeRTOS/SAME51 \
	-L$(LIBRARIES_DIR)/CANlib/SAME51_RTOS \
	-L$(LIBRARIES_DIR)/LibTinyusb/SAME5x \
	-L$(LIBRARIES_DIR)/LibMbedTls/SAME5x \
	-lCoreN2G \
	-lLibMbedTls \
	-lCANlib \
	-lRRFLibraries \
	-lFreeRTOS \
	-lLibTinyusb \
	-lsupc++

DUET3MINI_LDLIBS_POST := -Wl,--end-group -lm

# Object files - libcpp and libc first for proper symbol resolution
DUET3MINI_LIBCPP_OBJS := $(DUET3MINI_LIBCPP_SRCS:%.cpp=$(DUET3MINI_BUILD_DIR)/%.o)
DUET3MINI_LIBCPP_OBJS := $(DUET3MINI_LIBCPP_OBJS:%.cc=$(DUET3MINI_BUILD_DIR)/%.o)
DUET3MINI_LIBC_OBJS := $(DUET3MINI_LIBC_SRCS:%.c=$(DUET3MINI_BUILD_DIR)/%.o)
DUET3MINI_LIBC_OBJS := $(DUET3MINI_LIBC_OBJS:%.cpp=$(DUET3MINI_BUILD_DIR)/%.o)
DUET3MINI_CPP_OBJS := $(DUET3MINI_CPP_SRCS:%.cpp=$(DUET3MINI_BUILD_DIR)/%.o)
DUET3MINI_CPP_OBJS := $(DUET3MINI_CPP_OBJS:%.cc=$(DUET3MINI_BUILD_DIR)/%.o)
DUET3MINI_C_OBJS := $(DUET3MINI_C_SRCS:%.c=$(DUET3MINI_BUILD_DIR)/%.o)
DUET3MINI_OBJS := $(DUET3MINI_LIBCPP_OBJS) $(DUET3MINI_LIBC_OBJS) $(DUET3MINI_CPP_OBJS) $(DUET3MINI_C_OBJS)

# Dependency files
DUET3MINI_DEPS := $(DUET3MINI_OBJS:.o=.d)

# Target rule
.PHONY: Duet3Mini5plus
Duet3Mini5plus: $(DUET3MINI_TARGET_UF2)
	@echo "========================================"
	@echo "Duet3Mini5plus firmware build complete!"
	@echo "Output: $(DUET3MINI_TARGET_BIN)"
	@echo "UF2: $(DUET3MINI_TARGET_UF2)"
	@echo "========================================"
	@$(SIZE) $(DUET3MINI_TARGET_ELF)

# Link ELF file
$(DUET3MINI_TARGET_ELF): $(DUET3MINI_OBJS) $(DUET3MINI_COREN2G_LIB) $(DUET3MINI_RRFLIBS_LIB) $(DUET3MINI_FREERTOS_LIB) $(DUET3MINI_LIBTINYUSB_LIB) $(DUET3MINI_CANLIB_LIB) $(DUET3MINI_MBEDTLS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(DUET3MINI_LDFLAGS1) -o $@ $(DUET3MINI_LDFLAGS2) -Wl,--start-group $(DUET3MINI_OBJS) $(DUET3MINI_LDLIBS) $(DUET3MINI_LDLIBS_POST)

# Generate binary file
$(DUET3MINI_TARGET_BIN): $(DUET3MINI_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)command -v CrcAppender >/dev/null 2>&1 || { echo "CrcAppender not found on PATH" >&2; exit 1; }
	$(Q)echo "  CRC     $@"
	$(Q)CrcAppender $@

# Generate UF2 file for USB bootloader
$(DUET3MINI_TARGET_UF2): $(DUET3MINI_TARGET_BIN)
	$(Q)echo "  UF2     $@"
	$(Q)if [ -f Tools/uf2conv/uf2conv.py ]; then \
		python3 Tools/uf2conv/uf2conv.py -b 0x4000 -c -o $@ $<; \
	else \
		echo "uf2conv.py not found, skipping UF2 generation"; \
		echo "Binary file is available at: $<"; \
	fi

# Compile C++ files
$(DUET3MINI_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3MINI_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile C files
$(DUET3MINI_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(DUET3MINI_CFLAGS) -Dnoexcept= -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Compile .cc files (same as .cpp)
$(DUET3MINI_BUILD_DIR)/%.o: %.cc
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(DUET3MINI_CXXFLAGS) -DMQTTC_PAL_FILE="$(MQTTC_PAL_DEFINE)" -MMD -MP -o $@ $<

# Touch Version.cpp before build
.PHONY: duet3mini-prebuild
duet3mini-prebuild:
	@touch -c $(DUET3MINI_SRC_DIR)/Version.cpp 2>/dev/null || true

$(DUET3MINI_OBJS): | duet3mini-prebuild

# Include dependencies
-include $(DUET3MINI_DEPS)

# Clean target
.PHONY: clean-Duet3Mini5plus
clean-Duet3Mini5plus:
	@echo "Cleaning Duet3Mini5plus..."
	@rm -rf $(DUET3MINI_BUILD_DIR)

# Library dependencies (rules defined in main Makefile to avoid duplicates)
.PHONY: duet3mini-libs
duet3mini-libs: $(DUET3MINI_FREERTOS_LIB) $(DUET3MINI_COREN2G_LIB) $(DUET3MINI_RRFLIBS_LIB) $(DUET3MINI_LIBTINYUSB_LIB) $(DUET3MINI_MBEDTLS_LIB)
	@echo "All required libraries built successfully"

