# The RepRapFirmware boards: everything about DUET_BOARD apart from its MCU's toolchain file, which
# its preset selects. Included by the top-level CMakeLists.txt before the libraries are added,
# because it also sets the options they are built with.
#
# Variants of a board, which its presets set:
#   RRF_CAN0            use the MCU's CAN0 rather than CAN1 for CAN-FD
#   RRF_EMBEDDED_FILES  directory, under the project root, of files to embed after the image as its
#                       filesystem, in place of the SD card
#
# Library configuration:
#   DUET_LIBRARIES    the libraries to build
#   DUET_PART_DEFINE  macro naming the MCU part
#   DUET_RTOS, COREN2G_CAN, COREN2G_USB, COREN2G_SDHC
#
# Firmware:
#   RRF_MCU            the DUET_MCU the board is built for
#   RRF_OUTPUT         image file name, without extension
#   RRF_DEFINES        board macro, which selects the board's configuration headers, and any
#                      macros that select a variant of it
#   RRF_LINKER_SCRIPT  linker script, under src/
#   RRF_INCLUDES       extra include directories, under src/
#   RRF_DISPLAY        build the direct-connect LCD support in src/Display
#   RRF_UF2            also produce a .uf2 image for the board's USB bootloader

set(DUET_LIBRARIES
    FreeRTOS RRFLibraries CANlib CoreN2G LibTinyusb LibMbedTls WiFiSocketServerRTOS)

# Every RepRapFirmware board runs FreeRTOS and has CAN, USB and SD card support in CoreN2G
set(DUET_RTOS ON)
set(COREN2G_CAN ON)
set(COREN2G_USB ON)
set(COREN2G_SDHC ON)

option(RRF_CAN0 "Use CAN0 rather than CAN1 for CAN-FD" OFF)
set(RRF_EMBEDDED_FILES "" CACHE STRING
    "Directory, under the project root, of files to embed as the filesystem in place of the SD card")

set(RRF_INCLUDES)
set(RRF_DISPLAY OFF)
set(RRF_UF2 OFF)

if(DUET_BOARD MATCHES "^Duet3_(MB6HC|MB6XD)$")
    set(RRF_MCU SAME70)
    set(DUET_PART_DEFINE __SAME70Q20B__)
    set(RRF_LINKER_SCRIPT Hardware/SAME70/same70q20b_flash.ld)
    set(RRF_OUTPUT Duet3Firmware_${CMAKE_MATCH_1})
    set(RRF_DEFINES DUET3_${CMAKE_MATCH_1})
elseif(DUET_BOARD STREQUAL "Duet3Mini5plus")
    set(RRF_MCU SAME5x)
    set(DUET_PART_DEFINE __SAME54P20A__)
    set(RRF_OUTPUT Duet3Firmware_Mini5plus)
    set(RRF_DEFINES DUET3MINI_V04)
    set(RRF_LINKER_SCRIPT Hardware/SAME5x/same54p20a_flash_16k_bootloader.ld)
    set(RRF_INCLUDES Duet3Mini Hardware/SAME5x/Ethernet)
    set(RRF_DISPLAY ON)
    set(RRF_UF2 ON)
else()
    message(FATAL_ERROR "Unknown DUET_BOARD '${DUET_BOARD}': configure with a board's preset (cmake --list-presets)")
endif()

if(RRF_CAN0)
    string(APPEND RRF_OUTPUT _CAN0)
    list(APPEND RRF_DEFINES USE_CAN0)
endif()
if(RRF_EMBEDDED_FILES)
    list(APPEND RRF_DEFINES USE_EMBEDDED_FILES)
endif()

if(NOT DUET_MCU STREQUAL RRF_MCU)
    message(FATAL_ERROR "${DUET_BOARD} is a ${RRF_MCU} board, but this build tree is for ${DUET_MCU}: configure it with its preset")
endif()
