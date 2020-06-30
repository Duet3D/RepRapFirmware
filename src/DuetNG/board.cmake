add_definitions("-D__SAM4E8E__" "-DDUET_NG")
add_compile_options("-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")
add_link_options("-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

list(APPEND SRCS
    "${CMAKE_CURRENT_LIST_DIR}/DueXn.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Pins_DuetNG.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/SX1509.cpp"
)
list(APPEND INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}")

set(EXECUTABLE_NAME "Duet2CombinedFirmware")