# RepRapFirmware for the Duet 3 main boards, built as part of DuetFirmware against the shared
# libraries in lib/. A build tree builds one board, DUET_BOARD, as an executable target of the same
# name; Boards.cmake describes the boards. Its image goes to the top of the build tree: the .elf,
# the .bin with its CRC appended, the .map, and a .uf2 for boards whose bootloader takes one.

cmake_minimum_required(VERSION 3.25)
project(RepRapFirmware LANGUAGES C CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

string(TOLOWER "${CMAKE_HOST_SYSTEM_NAME}" _host_os)
string(REPLACE "darwin" "macos" _host_os "${_host_os}")
find_program(CRC_APPENDER CrcAppender
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Tools/CrcAppender/${_host_os}-${CMAKE_HOST_SYSTEM_PROCESSOR}"
    REQUIRED)

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

add_executable(${DUET_BOARD} ${_libcpp} ${_libc} ${_cxx} ${_c})

# Version.cpp holds the build date, so it is recompiled on every build
add_custom_target(rrf_build_date
    COMMAND "${CMAKE_COMMAND}" -E touch build_date.stamp
    BYPRODUCTS build_date.stamp)
set_source_files_properties(src/Version.cpp PROPERTIES
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/build_date.stamp")
add_dependencies(${DUET_BOARD} rrf_build_date)

set(_dir "${CMAKE_BINARY_DIR}")
set(_ld "${CMAKE_CURRENT_SOURCE_DIR}/src/${RRF_LINKER_SCRIPT}")
# RRF links libsupc++ for the C++ runtime and supplies the rest itself (src/libcpp), so the image
# is linked by the C driver rather than g++, which would add libstdc++
set_target_properties(${DUET_BOARD} PROPERTIES
    OUTPUT_NAME "${RRF_OUTPUT}"
    SUFFIX ".elf"
    RUNTIME_OUTPUT_DIRECTORY "${_dir}"
    LINKER_LANGUAGE C
    LINK_DEPENDS "${_ld}"
    ADDITIONAL_CLEAN_FILES "${_dir}/${RRF_OUTPUT}.map"
    C_STANDARD 99
    C_EXTENSIONS ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

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

target_compile_options(${DUET_BOARD} PRIVATE
    -fno-math-errno
    -fsingle-precision-constant
    -Wall
    -Wundef
    -Wdouble-promotion
    -Werror=return-type
    -Werror
    $<$<COMPILE_LANGUAGE:C>:-Werror=implicit;-Wwrite-strings>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics;-fno-rtti;-fexceptions;-Wfloat-conversion;-Wsuggest-override;-Wnoexcept;-Wshadow;-Wsign-promo;-fstack-usage>
    # __FILE__ relative to the project root, so the image does not depend on the checkout location
    -fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=)

# -static: there are no shared libraries. It also makes ld check each archive as it opens it, as it
# does for libraries found with -l; that changes the order of the veneers it generates for calls to
# code in RAM, so the image depends on it.
target_link_options(${DUET_BOARD} PRIVATE
    -static
    --specs=nosys.specs
    "-T${_ld}"
    "LINKER:-Map,${_dir}/${RRF_OUTPUT}.map"
    LINKER:--gc-sections
    LINKER:--entry=Reset_Handler
    LINKER:--cref
    LINKER:--check-sections
    LINKER:--fatal-warnings
    LINKER:--no-warn-rwx-segment
    LINKER:--unresolved-symbols=report-all
    LINKER:--warn-common
    LINKER:--warn-section-align
    LINKER:--warn-unresolved-symbols)

list(JOIN RRF_LINK_ORDER "," _group)
target_link_libraries(${DUET_BOARD} PRIVATE
    "$<LINK_GROUP:RESCAN,${_group},supc++>"
    m
    Duet::WiFiSocketServerRTOS)

set(_elf "$<TARGET_FILE:${DUET_BOARD}>")
set(_bin "${_dir}/${RRF_OUTPUT}.bin")
set(_images "${_bin}")
set(_files)
if(RRF_FILES)
    set(_files "${CMAKE_CURRENT_SOURCE_DIR}/${RRF_FILES}")
    file(GLOB_RECURSE _embedded CONFIGURE_DEPENDS "${_files}/*")
    # Relink when an embedded file changes, so the image is rebuilt with it
    set_property(TARGET ${DUET_BOARD} APPEND PROPERTY LINK_DEPENDS ${_embedded})
endif()
set(_uf2_command)
if(RRF_UF2)
    list(APPEND _images "${_dir}/${RRF_OUTPUT}.uf2")
    # The bootloader occupies the first 16K of flash
    set(_uf2_command COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/Tools/uf2conv/uf2conv.py"
        -b 0x4000 -c -o "${_dir}/${RRF_OUTPUT}.uf2" "${_bin}")
endif()
add_custom_command(TARGET ${DUET_BOARD} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary "${_elf}" "${_bin}"
    COMMAND "${CRC_APPENDER}" "${_bin}" ${_files}
    ${_uf2_command}
    COMMAND "${CMAKE_SIZE}" "${_elf}"
    BYPRODUCTS ${_images}
    COMMENT "Creating ${RRF_OUTPUT} images"
    VERBATIM)
