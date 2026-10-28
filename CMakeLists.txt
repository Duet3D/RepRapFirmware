# RepRapFirmware for the Duet 3 main boards, built as part of DuetFirmware against the shared
# libraries in lib/. A build tree builds one board, DUET_BOARD, as an executable target of the same
# name; Boards.cmake describes the boards. Its image goes to the top of the build tree: the .elf,
# the .bin with its CRC appended, the .map, and a .uf2 for boards whose bootloader takes one.

cmake_minimum_required(VERSION 3.25)
project(RepRapFirmware LANGUAGES C CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Sources, in link order: the replacement C++ and C runtime pieces come first so that theirs are
# the definitions used, then the firmware itself.
file(GLOB_RECURSE _libcpp CONFIGURE_DEPENDS src/libcpp/*.cpp src/libcpp/*.cc)
file(GLOB_RECURSE _libc   CONFIGURE_DEPENDS src/libc/*.c src/libc/*.cpp)
file(GLOB_RECURSE _cxx    CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE _c      CONFIGURE_DEPENDS src/*.c)
list(SORT _libcpp)
list(SORT _libc)
list(FILTER _cxx EXCLUDE REGEX "/src/(libcpp|libc)/")
list(FILTER _c   EXCLUDE REGEX "/src/libc/")

# Sources for other MCUs and boards, and parts of the bundled lwIP and MQTT-C that are not used
set(_excludes
    "/src/DuetNG/"
    "/src/Hardware/SAM4E/"
    "/src/Hardware/SAM4S/"
    "/src/Networking/W5500Ethernet/"
    "/src/Networking/LwipEthernet/Lwip/doc/"
    "/src/Networking/LwipEthernet/Lwip/src/apps/(http|lwiperf|mqtt|smtp|snmp|sntp|tftp)/"
    "/src/Networking/LwipEthernet/Lwip/src/netif/ppp/"
    "/src/Networking/MQTT/MQTT_C/examples/"
    "/src/Networking/MQTT/MQTT_C/tests\\.c$"
    "/src/Networking/MQTT/MQTT_C/src/mqtt_pal\\.c$")
if(DUET_MCU STREQUAL "SAME70")
    list(APPEND _excludes "/src/Hardware/SAME5x/")
else()
    list(APPEND _excludes "/src/Hardware/SAME70/")
endif()
if(NOT RRF_DISPLAY)
    list(APPEND _excludes "/src/Display/")
endif()
foreach(_ex IN LISTS _excludes)
    list(FILTER _cxx EXCLUDE REGEX "${_ex}")
    list(FILTER _c   EXCLUDE REGEX "${_ex}")
endforeach()

duet_firmware_executable(${DUET_BOARD}
    OUTPUT_NAME ${RRF_OUTPUT}
    LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/${RRF_LINKER_SCRIPT}"
    SOURCES ${_libcpp} ${_libc} ${_cxx} ${_c})
# Version.cpp holds the build date
duet_build_date(${DUET_BOARD} src/Version.cpp)
# RRF links libsupc++ for the C++ runtime and supplies the rest itself (src/libcpp), so the image
# is linked by the C driver rather than g++, which would add libstdc++
set_target_properties(${DUET_BOARD} PROPERTIES LINKER_LANGUAGE C)

list(TRANSFORM RRF_INCLUDES PREPEND "src/" OUTPUT_VARIABLE _includes)
target_include_directories(${DUET_BOARD} PRIVATE
    src
    src/Hardware/${DUET_MCU}
    src/Networking
    src/Networking/LwipEthernet/Lwip
    src/Networking/LwipEthernet/Lwip/src/include
    src/Networking/MQTT/MQTT_C/include
    ${_includes})

target_compile_definitions(${DUET_BOARD} PRIVATE
    ${DUET_PART_DEFINE}
    ${RRF_DEFINES}
    MQTTC_PAL_FILE=Networking/MQTT/mqtt_pal.h
    $<$<COMPILE_LANGUAGE:C>:noexcept=>
    $<$<COMPILE_LANGUAGE:CXX>:_XOPEN_SOURCE>)

set(_c_options
    -Werror=implicit
    -Wwrite-strings
    -fstack-usage)
set(_cxx_options
    -fno-threadsafe-statics
    -fno-rtti
    -fexceptions
    -Wfloat-conversion
    -Wsuggest-override
    -Wnoexcept
    -Wshadow
    -Wsign-promo
    -fstack-usage)

target_compile_options(${DUET_BOARD} PRIVATE
    -fno-math-errno
    -fsingle-precision-constant
    -Wall
    -Wundef
    -Wdouble-promotion
    -Werror=return-type
    -Werror
    "$<$<COMPILE_LANGUAGE:C>:${_c_options}>"
    "$<$<COMPILE_LANGUAGE:CXX>:${_cxx_options}>")

target_link_options(${DUET_BOARD} PRIVATE
    --specs=nosys.specs
    LINKER:--check-sections
    LINKER:--no-warn-rwx-segment
    LINKER:--unresolved-symbols=report-all
    LINKER:--warn-common
    LINKER:--warn-section-align
    LINKER:--warn-unresolved-symbols)

duet_link_libraries(${DUET_BOARD} supc++ m)

set(_files)
if(RRF_EMBEDDED_FILES)
    set(_files FILES "${CMAKE_CURRENT_SOURCE_DIR}/${RRF_EMBEDDED_FILES}")
endif()
duet_firmware_images(${DUET_BOARD} CRC ${_files})
if(RRF_UF2)
    # The bootloader occupies the first 16K of flash
    set(_bin "${CMAKE_BINARY_DIR}/${RRF_OUTPUT}.bin")
    set(_uf2 "${CMAKE_BINARY_DIR}/${RRF_OUTPUT}.uf2")
    add_custom_command(TARGET ${DUET_BOARD} POST_BUILD
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/Tools/uf2conv/uf2conv.py"
            -b 0x4000 -c -o "${_uf2}" "${_bin}"
        BYPRODUCTS "${_uf2}"
        VERBATIM)
endif()
