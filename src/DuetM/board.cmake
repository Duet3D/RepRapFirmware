add_definitions("-D__SAM4S8C__" "-DDUET_M")
add_compile_options("-mcpu=cortex-m4")
add_link_options("-mcpu=cortex-m4")

list(APPEND SRCS
    "${CMAKE_CURRENT_LIST_DIR}/Pins_DuetM.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Internet/DHCP/dhcp.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/W5500/w5500.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/WizSpi.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/socketlib.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/Wiznet/Ethernet/wizchip_conf.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/MdnsResponder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/W5500Interface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Networking/W5500Ethernet/W5500Socket.cpp"
)

list(APPEND INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}")

set(EXECUTABLE_NAME "DuetMaestroFirmware")