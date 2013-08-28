################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include arduino/variant/subdir.mk
-include arduino/core/USB/subdir.mk
-include arduino/core/subdir.mk
-include Libraries/SPI/subdir.mk
-include Libraries/SD/utility/subdir.mk
-include Libraries/SD/subdir.mk
-include Libraries/Ethernet/utility/subdir.mk
-include Libraries/Ethernet/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
/home/ensab/Pro/Git/RepRapFirmware/Release/arduino.ar \

ELF += \
RepRapFirmware.elf \

EEP += \
RepRapFirmware.eep \

FLASH_IMAGE += \
Release/RepRapFirmware.bin \


# All Target
all: RepRapFirmware

# Tool invocations
RepRapFirmware: $(FLASH_IMAGE)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-size" -A "/home/ensab/Pro/Git/RepRapFirmware/Release/RepRapFirmware.elf"
	@echo 'Finished building target: $@'
	@echo ' '

/home/ensab/Pro/Git/RepRapFirmware/Release/arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-ar" rcs "/home/ensab/Pro/Git/RepRapFirmware/Release/arduino.ar"   $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

RepRapFirmware.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++" -Os -Wl,--gc-sections -mcpu=cortex-m3 "-T/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x/linker_scripts/gcc/flash.ld" "-Wl,-Map,/home/ensab/Pro/Git/RepRapFirmware/Release/RepRapFirmware.map" -o "/home/ensab/Pro/Git/RepRapFirmware/Release/RepRapFirmware.elf" "-L/home/ensab/Pro/Git/RepRapFirmware/Release" -lm -lgcc -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols -Wl,--start-group "/home/ensab/Pro/Git/RepRapFirmware/Release/arduino/syscalls_sam3.c.o"   $(LINK_OBJ) $(AR)  "/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x/libsam_sam3x8e_gcc_rel.a" "/home/ensab/Pro/Git/RepRapFirmware/Release/arduino.ar" -Wl,--end-group
	@echo 'Finished building: $@'
	@echo ' '

RepRapFirmware.eep: $(ELF)
	@echo 'Create eeprom image'
	${A.RECIPE.OBJCOPY.EEP.PATTERN}
	@echo 'Finished building: $@'
	@echo ' '

Release/RepRapFirmware.bin: $(EEP)
	@echo 'Create Flash image (ihex format)'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-objcopy" -O binary "/home/ensab/Pro/Git/RepRapFirmware/Release/RepRapFirmware.elf" "/home/ensab/Pro/Git/RepRapFirmware/Release/RepRapFirmware.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(ELF)$(C++_DEPS)$(AR)$(C_DEPS)$(CC_DEPS)$(FLASH_IMAGE)$(AR_OBJ)$(CPP_DEPS)$(LINK_OBJ)$(EEP)$(CXX_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) RepRapFirmware
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
