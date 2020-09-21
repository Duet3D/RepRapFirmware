if(NOT DuetWiFiSocketServer_FOUND)
    message(FATAL_ERROR "DuetNG build requires DuetWiFiSocketServer")
endif()

add_definitions("-D__SAM4E8E__" "-DDUET_NG")
add_compile_options("-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")
add_link_options("-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

list(APPEND SRCS
    "${CMAKE_CURRENT_LIST_DIR}/DueXn.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Pins_DuetNG.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/SX1509.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/ESP8266WiFi/WiFiInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/ESP8266WiFi/WiFiSocket.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/ESP8266WiFi/WifiFirmwareUploader.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Internet/DHCP/dhcp.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/W5500/w5500.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/WizSpi.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/socketlib.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/wizchip_conf.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/MdnsResponder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/W5500Interface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/W5500Socket.cpp"
)

list(APPEND INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}"
                          "${DuetWiFiSocketServer_DIR}/src/include")

set(EXECUTABLE_NAME "Duet2CombinedFirmware")