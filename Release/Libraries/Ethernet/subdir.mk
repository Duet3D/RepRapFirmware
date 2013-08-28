################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/Dhcp.cpp \
/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/Dns.cpp \
/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/Ethernet.cpp \
/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/EthernetClient.cpp \
/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/EthernetServer.cpp \
/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/EthernetUdp.cpp 

CPP_DEPS += \
./Libraries/Ethernet/Dhcp.cpp.d \
./Libraries/Ethernet/Dns.cpp.d \
./Libraries/Ethernet/Ethernet.cpp.d \
./Libraries/Ethernet/EthernetClient.cpp.d \
./Libraries/Ethernet/EthernetServer.cpp.d \
./Libraries/Ethernet/EthernetUdp.cpp.d 

LINK_OBJ += \
./Libraries/Ethernet/Dhcp.cpp.o \
./Libraries/Ethernet/Dns.cpp.o \
./Libraries/Ethernet/Ethernet.cpp.o \
./Libraries/Ethernet/EthernetClient.cpp.o \
./Libraries/Ethernet/EthernetServer.cpp.o \
./Libraries/Ethernet/EthernetUdp.cpp.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Ethernet/Dhcp.cpp.o: /usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/Dhcp.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++" -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/libsam" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/CMSIS/Include/" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/"   -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/cores/arduino" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/utility" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/SPI" -I"/usr/local/arduino-1.5.2/libraries/SD" -I"/usr/local/arduino-1.5.2/libraries/SD/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

Libraries/Ethernet/Dns.cpp.o: /usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/Dns.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++" -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/libsam" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/CMSIS/Include/" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/"   -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/cores/arduino" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/utility" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/SPI" -I"/usr/local/arduino-1.5.2/libraries/SD" -I"/usr/local/arduino-1.5.2/libraries/SD/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

Libraries/Ethernet/Ethernet.cpp.o: /usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/Ethernet.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++" -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/libsam" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/CMSIS/Include/" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/"   -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/cores/arduino" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/utility" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/SPI" -I"/usr/local/arduino-1.5.2/libraries/SD" -I"/usr/local/arduino-1.5.2/libraries/SD/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

Libraries/Ethernet/EthernetClient.cpp.o: /usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/EthernetClient.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++" -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/libsam" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/CMSIS/Include/" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/"   -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/cores/arduino" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/utility" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/SPI" -I"/usr/local/arduino-1.5.2/libraries/SD" -I"/usr/local/arduino-1.5.2/libraries/SD/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

Libraries/Ethernet/EthernetServer.cpp.o: /usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/EthernetServer.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++" -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/libsam" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/CMSIS/Include/" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/"   -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/cores/arduino" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/utility" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/SPI" -I"/usr/local/arduino-1.5.2/libraries/SD" -I"/usr/local/arduino-1.5.2/libraries/SD/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

Libraries/Ethernet/EthernetUdp.cpp.o: /usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/EthernetUdp.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/usr/local/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++" -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/libsam" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/CMSIS/Include/" "-I/usr/local/arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/"   -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/cores/arduino" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/variants/arduino_due_x" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/Ethernet/utility" -I"/usr/local/arduino-1.5.2/hardware/arduino/sam/libraries/SPI" -I"/usr/local/arduino-1.5.2/libraries/SD" -I"/usr/local/arduino-1.5.2/libraries/SD/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


