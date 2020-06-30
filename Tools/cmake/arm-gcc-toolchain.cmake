set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# Common flags for ARM targets
set(CMAKE_C_FLAGS_INIT "-mthumb")
set(CMAKE_CXX_FLAGS_INIT "-mthumb")

# Needed to pass compiler check
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs -mthumb")