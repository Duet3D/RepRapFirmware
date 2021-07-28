cmake_minimum_required( VERSION 3.10 )

#configure_file(src/config.h.in src/config.h)

# skip compiler build tests
set ( CMAKE_C_COMPILER_FORCED True )
set ( CMAKE_CXX_COMPILER_FORCED True )

set( DEVICE "mini5+" CACHE STRING "choose a hardware device mini5+, ..." )


if ("${DEVICE}" STREQUAL "mini5+" )
	#set( SYSTEM_FILE ${SYSTEM_FILE_SAM3S4} )
	#set( STARTUP_FILE ${STARTUP_FILE_SAM3S4} )
	#set( EXCEPTION_FILE ${EXCEPTION_FILE_SAM3S4} )
	set ( DEVICE_FLAGS "-D__SAME54P20A__ -DDUET3MINI_V04=1 -D_XOPEN_SOURCE -DRTOS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard" )
	set ( DEVICE_INCLUDE "" )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEVICE_FLAGS}" )
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEVICE_FLAGS}" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM3S4}" )
elseif ("${DEVICE}" STREQUAL "v2-5.0" )
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM3S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM3S4} )
	set( EXCEPTION_FILE ${EXCEPTION_FILE_SAM3S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM3S4}" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM3S4}" )
else()
	message(FATAL_ERROR "Please configure a device cmake -DDEVICE=mini5+")
endif()

set( CMAKE_TOOLCHAIN_FILE toolchains/arm-none-eabi.cmake )

message( "RRF C_FLAGS: " ${CMAKE_C_FLAGS} )
message( "RRF CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )
message( "RRF ASM_FLAGS: " ${CMAKE_ASM_FLAGS} )

message("Device: " ${DEVICE})

set ( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static --specs=nosys.specs -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--fatal-warnings -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -T${CMAKE_CURRENT_SOURCE_DIR}/src/Hardware/SAME5x/same54p20a_flash.ld" )
#set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static --specs=nano.specs -Wl,--gc-sections -Wl,--fatal-warnings" )
message( "LD_FLAGS:" ${CMAKE_EXE_LINKER_FLAGS} )


project( reprap
	DESCRIPTION "This is RepRap Firmware"
	HOMEPAGE_URL "https://github.com/Duet3D/RepRapFirmware"
)

add_subdirectory( lib/FreeRTOS )
add_subdirectory( lib/libCAN )
add_subdirectory( lib/libCoreN2G )
add_subdirectory( lib/librrf )
add_executable( reprap.elf
	${DEVICE_FILES}
	src/Accelerometers/Accelerometers.cpp
	src/Accelerometers/LIS3DH.cpp
	src/CAN/CanInterface.cpp
	src/CAN/CanMessageGenericConstructor.cpp
	src/CAN/CanMotion.cpp
	src/CAN/CommandProcessor.cpp
	src/CAN/ExpansionManager.cpp
	src/Comms/AuxDevice.cpp
	src/Comms/FirmwareUpdater.cpp
	src/Comms/PanelDueUpdater.cpp
	src/Display/Display.cpp
	src/Display/Lcd/Fonts/glcd11x14.cpp
	src/Display/Lcd/Fonts/glcd7x11.cpp
	src/Display/Lcd/Lcd.cpp
	src/Display/Lcd/ST7567/Lcd7567.cpp
	src/Display/Lcd/ST7920/Lcd7920.cpp
	src/Display/Menu.cpp
	src/Display/MenuItem.cpp
	src/Display/RotaryEncoder.cpp
	src/Duet3Mini/Pins_Duet3Mini.cpp
	#src/Duet3_V06/Pins_Duet3_V06.cpp
	src/Endstops/Endstop.cpp
	src/Endstops/EndstopsManager.cpp
	src/Endstops/LocalZProbe.cpp
	src/Endstops/RemoteZProbe.cpp
	src/Endstops/StallDetectionEndstop.cpp
	src/Endstops/SwitchEndstop.cpp
	src/Endstops/ZProbe.cpp
	src/Endstops/ZProbeEndstop.cpp
	src/Fans/Fan.cpp
	src/Fans/FansManager.cpp
	src/Fans/LedStripDriver.cpp
	src/Fans/LocalFan.cpp
	src/Fans/RemoteFan.cpp
	src/FilamentMonitors/Duet3DFilamentMonitor.cpp
	src/FilamentMonitors/FilamentMonitor.cpp
	src/FilamentMonitors/LaserFilamentMonitor.cpp
	src/FilamentMonitors/PulsedFilamentMonitor.cpp
	src/FilamentMonitors/RotatingMagnetFilamentMonitor.cpp
	src/FilamentMonitors/SimpleFilamentMonitor.cpp
	src/GCodes/GCodeBuffer/BinaryParser.cpp
	src/GCodes/GCodeBuffer/ExpressionParser.cpp
	src/GCodes/GCodeBuffer/GCodeBuffer.cpp
	src/GCodes/GCodeBuffer/StringParser.cpp
	src/GCodes/GCodeException.cpp
	src/GCodes/GCodeFileInfo.cpp
	src/GCodes/GCodeInput.cpp
	src/GCodes/GCodeMachineState.cpp
	src/GCodes/GCodeQueue.cpp
	src/GCodes/GCodes.cpp
	src/GCodes/GCodes2.cpp
	src/GCodes/GCodes3.cpp
	src/GCodes/GCodes4.cpp
	src/GCodes/ObjectTracker.cpp
	src/GCodes/RestorePoint.cpp
	src/GCodes/StraightProbeSettings.cpp
	src/GCodes/Trigger.cpp
	src/GPIO/GpInPort.cpp
	src/GPIO/GpOutPort.cpp
	src/Hardware/ExceptionHandlers.cpp
	src/Hardware/I2C.cpp
	src/Hardware/IoPorts.cpp
	src/Hardware/NonVolatileMemory.cpp
	src/Hardware/SAME5x/Devices.cpp
	src/Hardware/SAME5x/Ethernet/GmacInterface.cpp
	src/Hardware/SAME5x/Ethernet/gmac_phy.c
	src/Hardware/SAME5x/Ethernet/ksz8081rna/ethernet_phy.c
	src/Hardware/SAME5x/Main.cpp
	src/Hardware/SharedSpi/SharedSpiClient.cpp
	src/Hardware/SharedSpi/SharedSpiDevice.cpp
	src/Hardware/SoftwareReset.cpp
	src/Heating/FOPDT.cpp
	src/Heating/Heat.cpp
	src/Heating/Heater.cpp
	src/Heating/HeaterMonitor.cpp
	src/Heating/LocalHeater.cpp
	src/Heating/RemoteHeater.cpp
	src/Heating/Sensors/AdditionalOutputSensor.cpp
	src/Heating/Sensors/CpuTemperatureSensor.cpp
	src/Heating/Sensors/CurrentLoopTemperatureSensor.cpp
	src/Heating/Sensors/DhtSensor.cpp
	src/Heating/Sensors/LinearAnalogSensor.cpp
	src/Heating/Sensors/RemoteSensor.cpp
	src/Heating/Sensors/RtdSensor31865.cpp
	src/Heating/Sensors/SensorWithPort.cpp
	src/Heating/Sensors/SpiTemperatureSensor.cpp
	src/Heating/Sensors/TemperatureSensor.cpp
	src/Heating/Sensors/Thermistor.cpp
	src/Heating/Sensors/ThermocoupleSensor31855.cpp
	src/Heating/Sensors/ThermocoupleSensor31856.cpp
	src/Heating/Sensors/TmcDriverTemperatureSensor.cpp
	src/Heating/TemperatureError.cpp
	src/InputMonitors/InputMonitor.cpp
	src/Libraries/Fatfs/diskio.cpp
	src/Libraries/Fatfs/fattime_rtc.cpp
	src/Libraries/Fatfs/ff.c
	src/Libraries/Fatfs/ffunicode.c
	src/Libraries/sd_mmc/ctrl_access.c
	src/Libraries/sd_mmc/sd_mmc.c
	src/Libraries/sd_mmc/sd_mmc_mem.c
	src/Libraries/sd_mmc/sd_mmc_spi.cpp
	src/Libraries/sha1/sha1.c
	src/Linux/DataTransfer.cpp
	src/Linux/LinuxInterface.cpp
	src/Movement/AxisShaper.cpp
	src/Movement/BedProbing/Grid.cpp
	src/Movement/BedProbing/RandomProbePointSet.cpp
	src/Movement/DDA.cpp
	src/Movement/DDARing.cpp
	src/Movement/DriveMovement.cpp
	src/Movement/ExtruderShaper.cpp
	src/Movement/HeightControl/HeightController.cpp
	src/Movement/Kinematics/CoreKinematics.cpp
	src/Movement/Kinematics/FiveBarScaraKinematics.cpp
	src/Movement/Kinematics/HangprinterKinematics.cpp
	src/Movement/Kinematics/Kinematics.cpp
	src/Movement/Kinematics/LinearDeltaKinematics.cpp
	src/Movement/Kinematics/PolarKinematics.cpp
	src/Movement/Kinematics/RotaryDeltaKinematics.cpp
	src/Movement/Kinematics/RoundBedKinematics.cpp
	src/Movement/Kinematics/ScaraKinematics.cpp
	src/Movement/Kinematics/ZLeadscrewKinematics.cpp
	src/Movement/Move.cpp
	src/Movement/MoveSegment.cpp
	src/Movement/RawMove.cpp
	src/Movement/StepTimer.cpp
	src/Movement/StepperDrivers/DriverMode.cpp
	src/Movement/StepperDrivers/TMC22xx.cpp
	src/Movement/StepperDrivers/TMC2660.cpp
	src/Movement/StepperDrivers/TMC51xx.cpp
	src/Networking/ESP8266WiFi/WiFiInterface.cpp
	src/Networking/ESP8266WiFi/WiFiSocket.cpp
	src/Networking/ESP8266WiFi/WifiFirmwareUploader.cpp
	src/Networking/FtpResponder.cpp
	src/Networking/HttpResponder.cpp
	src/Networking/LwipEthernet/GMAC/ethernet_sam.cpp
	src/Networking/LwipEthernet/Lwip/src/Filelists.cmake
	src/Networking/LwipEthernet/Lwip/src/api/api_lib.c
	src/Networking/LwipEthernet/Lwip/src/api/api_msg.c
	src/Networking/LwipEthernet/Lwip/src/api/err.c
	src/Networking/LwipEthernet/Lwip/src/api/if_api.c
	src/Networking/LwipEthernet/Lwip/src/api/netbuf.c
	src/Networking/LwipEthernet/Lwip/src/api/netdb.c
	src/Networking/LwipEthernet/Lwip/src/api/netifapi.c
	src/Networking/LwipEthernet/Lwip/src/api/sockets.c
	src/Networking/LwipEthernet/Lwip/src/api/tcpip.c
	src/Networking/LwipEthernet/Lwip/src/apps/mdns/mdns.c
	src/Networking/LwipEthernet/Lwip/src/apps/netbiosns/netbiosns.c
	src/Networking/LwipEthernet/Lwip/src/core/altcp.c
	src/Networking/LwipEthernet/Lwip/src/core/altcp_alloc.c
	src/Networking/LwipEthernet/Lwip/src/core/altcp_tcp.c
	src/Networking/LwipEthernet/Lwip/src/core/def.c
	src/Networking/LwipEthernet/Lwip/src/core/dns.c
	src/Networking/LwipEthernet/Lwip/src/core/inet_chksum.c
	src/Networking/LwipEthernet/Lwip/src/core/init.c
	src/Networking/LwipEthernet/Lwip/src/core/ip.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/autoip.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/dhcp.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/etharp.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/icmp.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/igmp.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/ip4.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/ip4_addr.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv4/ip4_frag.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/dhcp6.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/ethip6.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/icmp6.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/inet6.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/ip6.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/ip6_addr.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/ip6_frag.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/mld6.c
	src/Networking/LwipEthernet/Lwip/src/core/ipv6/nd6.c
	src/Networking/LwipEthernet/Lwip/src/core/mem.c
	src/Networking/LwipEthernet/Lwip/src/core/memp.c
	src/Networking/LwipEthernet/Lwip/src/core/netif.c
	src/Networking/LwipEthernet/Lwip/src/core/pbuf.c
	src/Networking/LwipEthernet/Lwip/src/core/raw.c
	src/Networking/LwipEthernet/Lwip/src/core/stats.c
	src/Networking/LwipEthernet/Lwip/src/core/sys.c
	src/Networking/LwipEthernet/Lwip/src/core/tcp.c
	src/Networking/LwipEthernet/Lwip/src/core/tcp_in.c
	src/Networking/LwipEthernet/Lwip/src/core/tcp_out.c
	src/Networking/LwipEthernet/Lwip/src/core/timeouts.c
	src/Networking/LwipEthernet/Lwip/src/core/udp.c
	#src/Networking/LwipEthernet/Lwip/src/include/lwip/init.h.cmake.in
	src/Networking/LwipEthernet/Lwip/src/netif/bridgeif.c
	src/Networking/LwipEthernet/Lwip/src/netif/bridgeif_fdb.c
	src/Networking/LwipEthernet/Lwip/src/netif/ethernet.c
	src/Networking/LwipEthernet/Lwip/src/netif/lowpan6.c
	src/Networking/LwipEthernet/Lwip/src/netif/lowpan6_ble.c
	src/Networking/LwipEthernet/Lwip/src/netif/lowpan6_common.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/auth.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/ccp.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/chap-md5.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/chap-new.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/chap_ms.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/demand.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/eap.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/ecp.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/eui64.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/fsm.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/ipcp.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/ipv6cp.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/lcp.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/magic.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/mppe.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/multilink.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/polarssl/arc4.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/polarssl/des.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/polarssl/md4.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/polarssl/md5.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/polarssl/sha1.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/ppp.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/pppapi.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/pppcrypt.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/pppoe.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/pppol2tp.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/pppos.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/upap.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/utils.c
	src/Networking/LwipEthernet/Lwip/src/netif/ppp/vj.c
	src/Networking/LwipEthernet/Lwip/src/netif/slipif.c
	src/Networking/LwipEthernet/Lwip/src/netif/zepif.c
	src/Networking/LwipEthernet/LwipEthernetInterface.cpp
	src/Networking/LwipEthernet/LwipSocket.cpp
	src/Networking/Network.cpp
	src/Networking/NetworkBuffer.cpp
	src/Networking/NetworkInterface.cpp
	src/Networking/NetworkResponder.cpp
	src/Networking/TelnetResponder.cpp
	src/Networking/UploadingNetworkResponder.cpp
	src/ObjectModel/GlobalVariables.cpp
	src/ObjectModel/ObjectModel.cpp
	src/ObjectModel/Variable.cpp
	#src/Pccb/Pins_Pccb.cpp
	src/Platform/Heap.cpp
	src/Platform/Logger.cpp
	src/Platform/OutputMemory.cpp
	src/Platform/Platform.cpp
	src/Platform/PortControl.cpp
	src/Platform/RepRap.cpp
	src/Platform/Roland.cpp
	src/Platform/Scanner.cpp
	src/Platform/Tasks.cpp
	src/PrintMonitor/PrintMonitor.cpp
	src/RepRapFirmware.cpp
	src/Storage/CRC16.cpp
	src/Storage/CRC32.cpp
	src/Storage/FileInfoParser.cpp
	src/Storage/FileStore.cpp
	src/Storage/MassStorage.cpp
	src/Tools/Filament.cpp
	src/Tools/Spindle.cpp
	src/Tools/Tool.cpp
	src/bossa/Applet.cpp
	src/bossa/BossaFlash.cpp
	src/bossa/Device.cpp
	src/bossa/EefcFlash.cpp
	src/bossa/Flasher.cpp
	src/bossa/Samba.cpp
	src/bossa/WordCopyApplet.cpp
	src/bossa/WordCopyArm.cpp
	src/libc/memcmp.c
	src/libc/memcpy.c
	src/libc/memmove.c
	src/libc/memset.c
	src/libc/nano-mallocr.c
	src/libc/strptime.cpp
	src/libcpp/eh_alloc.cpp
	src/libcpp/vterminate.cc
)


target_include_directories( reprap.elf PUBLIC
	src

	/home/michi/work/duet3d/rrf/DuetWiFiSocketServer/src/include

	src/Hardware/SAME5x
	src/Hardware/SAME5x/Ethernet

	src/Networking
	src/Networking/LwipEthernet/Lwip
	src/Networking/LwipEthernet/Lwip/src/include
	src/UI
	src/config
)

target_link_libraries( reprap.elf rrf CoreN2G CAN FreeRTOS supc++ m )

add_custom_target( reprap.bin ALL
	DEPENDS reprap.elf
	COMMAND arm-none-eabi-objcopy -O binary reprap.elf reprap.bin
)
