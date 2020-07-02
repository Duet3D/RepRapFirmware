add_definitions("-D__SAM4S8C__" "-DDUET_M")
add_compile_options("-mcpu=cortex-m4")
add_link_options("-mcpu=cortex-m4")

list(APPEND SRCS
    "${CMAKE_CURRENT_LIST_DIR}/Pins_DuetM.cpp"
)

list(REMOVE_ITEM SRCS
    "src/Networking/ESP8266WiFi/WiFiInterface.cpp"
    "src/Networking/ESP8266WiFi/WiFiSocket.cpp"
    "src/Networking/ESP8266WiFi/WifiFirmwareUploader.cpp"
)

list(APPEND INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}")

set(EXECUTABLE_NAME "DuetMaestroFirmware")