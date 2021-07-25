cmake_minimum_required( VERSION 3.10 )

#configure_file(src/config.h.in src/config.h)

# skip compiler build tests
set ( CMAKE_C_COMPILER_FORCED True )
set ( CMAKE_CXX_COMPILER_FORCED True )

# all v2 linker script
# src/ASF/sam/utils/linker_scripts/sam3s/sam3s4/gcc/flash.ld"
# all v2 compiler flags
# -std=gnu++17 -DNDEBUG -DBOARD=USER_BOARD -DARM_MATH_CM3=true -D__SAM3S4B__
# v2 differences compiler flags
# -DSCREEN_43 -DSCREEN_50 -DSCREEN_70 -DSCREEN_70CPLD

# all v3 linker script
# src/ASF/sam/utils/linker_scripts/sam4s/sam4s4/gcc/flash.ld
# all v3 compiler flags
# -std=gnu++17 -DNDEBUG -D__SAM4S4B__ -DBOARD=USER_BOARD -DARM_MATH_CM4=true
# v3 differences compiler flags
# -DSCREEN_43 -DSCREEN_50 -DSCREEN_70 -DSCREEN_70CPLD

# 5.0i and 7.0i linker script
# src/ASF/sam/utils/linker_scripts/sam4s/sam4s4/gcc/flash.ld
# 5.0i and 7.0i compiler flags
# std=gnu++17 -DNDEBUG -D__SAM4S4B__ -DBOARD=USER_BOARD -DARM_MATH_CM4=true
# 5.0i and 7.0i differences
# -DSCREEN_50E -DSCREEN_70E

set( CMAKE_C_FLAGS "-O2 -g -DNDEBUG -DBOARD=USER_BOARD" )

file( REAL_PATH "src/ASF/sam/utils/linker_scripts/sam3s/sam3s4/gcc/flash.ld" LINKER_FILE_SAM3S4 )
file( REAL_PATH "src/ASF/sam/utils/linker_scripts/sam4s/sam4s4/gcc/flash.ld" LINKER_FILE_SAM4S4 )

set( SYSTEM_FILE_SAM3S4 "src/ASF/sam/utils/cmsis/sam3s/source/templates/system_sam3s.c" )
set( STARTUP_FILE_SAM3S4 "src/ASF/sam/utils/cmsis/sam3s/source/templates/gcc/startup_sam3s.c" )
set( EXCEPTION_FILE_SAM3S4 "src/ASF/sam/utils/cmsis/sam3s/source/templates/exceptions.c" )
set( C_FLAGS_SAM3S4 "-DARM_MATH_CM3=true -D__SAM3S4B__ -mcpu=cortex-m3 -mthumb" )
set( LD_FLAGS_SAM3S4 "-Wl,--defsym,__stack_size__=0x1000 -T${LINKER_FILE_SAM3S4} -Wl,-Map,paneldue.map" )

set( SYSTEM_FILE_SAM4S4 "src/ASF/sam/utils/cmsis/sam4s/source/templates/system_sam4s.c" )
set( STARTUP_FILE_SAM4S4 "src/ASF/sam/utils/cmsis/sam4s/source/templates/gcc/startup_sam4s.c" )
set( C_FLAGS_SAM4S4 "-DARM_MATH_CM4=true -D__SAM4S4B__ -mcpu=cortex-m4 -mthumb" )
set( LD_FLAGS_SAM4S4 "-Wl,--defsym,__stack_size__=0x1000 -T${LINKER_FILE_SAM4S4} -Wl,-Map,paneldue.map" )

if ("${DEVICE}" STREQUAL "v2-4.3")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM3S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM3S4} )
	set( EXCEPTION_FILE ${EXCEPTION_FILE_SAM3S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM3S4} -DSCREEN_43=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM3S4}" )
elseif ("${DEVICE}" STREQUAL "v2-5.0")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM3S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM3S4} )
	set( EXCEPTION_FILE ${EXCEPTION_FILE_SAM3S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM3S4} -DSCREEN_50=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM3S4}" )
elseif ("${DEVICE}" STREQUAL "v2-7.0")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM3S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM3S4} )
	set( EXCEPTION_FILE ${EXCEPTION_FILE_SAM3S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM3S4} -DSCREEN_70=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM3S4}" )
elseif ("${DEVICE}" STREQUAL "v2-7.0c")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM3S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM3S4} )
	set( EXCEPTION_FILE ${EXCEPTION_FILE_SAM3S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM3S4} -DSCREEN_70CPLD=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM3S4}" )
elseif ("${DEVICE}" STREQUAL "v3-4.3")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM4S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM4S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM4S4} -DSCREEN_43=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM4S4}" )
elseif ("${DEVICE}" STREQUAL "v3-5.0")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM4S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM4S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM4S4} -DSCREEN_50=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM4S4}" )
elseif ("${DEVICE}" STREQUAL "v3-7.0")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM4S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM4S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM4S4} -DSCREEN_70=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM4S4}" )
elseif ("${DEVICE}" STREQUAL "v3-7.0c")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM4S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM4S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM4S4} -DSCREEN_70CPLD=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM4S4}" )
elseif ("${DEVICE}" STREQUAL "5.0i")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM4S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM4S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM4S4} -DSCREEN_50E=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM4S4}" )
elseif ("${DEVICE}" STREQUAL "7.0i")
	set( SYSTEM_FILE ${SYSTEM_FILE_SAM4S4} )
	set( STARTUP_FILE ${STARTUP_FILE_SAM4S4} )
	set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_SAM4S4} -DSCREEN_70E=1" )
	set( CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS_SAM4S4}" )
else()
	message(FATAL_ERROR "Please configure a device cmake -DDEVICE=v2-4.3|v2-5.0|v2-7.0|v2-7.0c|v3-4.3|v3-5.0|v3-7.0|v3-7.0c|5.0i|7.0i")
endif()

message("Device: " ${DEVICE})
set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static --specs=nano.specs -Wl,--gc-sections -Wl,--fatal-warnings" )

set( CMAKE_TOOLCHAIN_FILE toolchains/arm-none-eabi.cmake )

project( paneldue VERSION 3.4 )
set( CMAKE_PROJECT_DESCRIPTION "This is PanelDue Firmware" )
set( CMAKE_PROJECT_HOMEPAGE_URL "https://github.com/Duet3D/PanelDueFirmware" )

add_subdirectory( lib/librrf )
add_executable( paneldue.elf
	${SYSTEM_FILE}
	${STARTUP_FILE}
	${EXCEPTION_FILE}
	src/ASF/common/boards/user_board/init.c
	src/ASF/common/services/clock/sam4s/sysclk.c
	src/ASF/common/services/delay/sam/cycle_counter.c
	src/ASF/common/utils/interrupt/interrupt_sam_nvic.c
	src/ASF/sam/drivers/chipid/chipid.c
	src/ASF/sam/drivers/efc/efc.c
	src/ASF/sam/drivers/matrix/matrix.c
	src/ASF/sam/drivers/pio/pio.c
	src/ASF/sam/drivers/pio/pio_handler.c
	src/ASF/sam/drivers/pmc/pmc.c
	src/ASF/sam/drivers/pwm/pwm.c
	src/ASF/sam/drivers/rstc/rstc.c
	src/ASF/sam/drivers/uart/uart.c
	src/ASF/sam/drivers/wdt/wdt.c
	src/ASF/sam/services/flash_efc/flash_efc.c
	src/ASF/sam/utils/syscalls/gcc/syscalls.c
	src/FileManager.cpp
	src/FlashData.cpp
	src/Fonts/glcd19x21.cpp
	src/Fonts/glcd28x32.cpp
	src/Hardware/Backlight.cpp
	src/Hardware/Buzzer.cpp
	src/Hardware/FlashStorage.cpp
	src/Hardware/Mem.cpp
	src/Hardware/OneBitPort.cpp
	src/Hardware/Reset.cpp
	src/Hardware/RotaryEncoder.cpp
	src/Hardware/SerialIo.cpp
	src/Hardware/SysTick.cpp
	src/Hardware/UTFT.cpp
	src/Hardware/UTouch.cpp
	src/Icons/HomeIcons.cpp
	src/Icons/KeyIcons.cpp
	src/Icons/MiscIcons.cpp
	src/Icons/NozzleIcons.cpp
	src/Library/Misc.cpp
	src/ObjectModel/Axis.cpp
	src/ObjectModel/BedOrChamber.cpp
	src/ObjectModel/Spindle.cpp
	src/ObjectModel/Tool.cpp
	src/ObjectModel/Utils.cpp
	src/PanelDue.cpp
	src/RequestTimer.cpp
	src/UI/ColourSchemes.cpp
	src/UI/Display.cpp
	src/UI/MessageLog.cpp
	src/UI/UserInterface.cpp
)


target_include_directories( paneldue.elf PUBLIC
	src
	src/ASF/common/boards
	src/ASF/common/boards/user_board
	src/ASF/common/services/clock
	src/ASF/common/services/delay
	src/ASF/common/utils
	src/ASF/sam/drivers
	src/ASF/sam/drivers/chipid
	src/ASF/sam/drivers/efc
	src/ASF/sam/drivers/efc
	src/ASF/sam/drivers/matrix
	src/ASF/sam/drivers/pio
	src/ASF/sam/drivers/pmc
	src/ASF/sam/drivers/pwm
	src/ASF/sam/drivers/rstc
	src/ASF/sam/drivers/uart
	src/ASF/sam/drivers/wdt
	src/ASF/sam/services/flash_efc
	src/ASF/sam/utils
	src/ASF/sam/utils/cmsis/sam3s/include
	src/ASF/sam/utils/cmsis/sam3s/source/templates
	src/ASF/sam/utils/cmsis/sam4s/include
	src/ASF/sam/utils/cmsis/sam4s/source/templates
	src/ASF/sam/utils/header_files
	src/ASF/sam/utils/preprocessor
	src/ASF/thirdparty/CMSIS/Include
	src/UI
	src/config
)

target_link_libraries( paneldue.elf rrf )

add_custom_target( paneldue.bin ALL
	DEPENDS paneldue.elf
	COMMAND arm-none-eabi-objcopy -O binary paneldue.elf paneldue.bin
)
add_custom_target( paneldue-logo.bin ALL
	DEPENDS paneldue.bin ${PROJECT_SOURCE_DIR}/SplashScreens/SplashScreen-Duet3D-800x480.bin
	COMMAND cat paneldue.bin ${PROJECT_SOURCE_DIR}/SplashScreens/SplashScreen-Duet3D-800x480.bin > paneldue-logo.bin
)

