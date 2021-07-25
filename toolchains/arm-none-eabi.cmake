#include( CMakeForceCompiler )

# usage
# cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/rv32imac.cmake ../

# Look for GCC in path
# https://xpack.github.io/arm-none-eabi-gcc/
FIND_FILE( ARM_NONE_EABI_GCC_COMPILER_EXE "arm-none-eabi-gcc.exe" PATHS ENV INCLUDE )
FIND_FILE( ARM_NONE_EABI_GCC_COMPILER "arm-none-eabi-gcc" PATHS ENV INCLUDE )

# Look for ARM-NONE-EABI github GCC
# https://github.com/riscv/riscv-gnu-toolchain
FIND_FILE( ARM_NONE_EABI_GCC_COMPILER_EXT "arm-none-eabi-gcc.exe" PATHS ENV INCLUDE )
FIND_FILE( ARM_NONE_EABI_GCC_COMPILER "arm-none-eabi-gcc" PATHS ENV INCLUDE )

# Select which is found
if ( EXISTS ${ARM_NONE_EABI_GCC_COMPILER} )
  set( ARM_NONE_EABI_GCC_COMPILER ${ARM_NONE_EABI_GCC_COMPILER} )
elseif ( EXISTS ${ARM_NONE_EABI_GCC_COMPILER_EXE} )
  set( ARM_NONE_EABI_GCC_COMPILER ${ARM_NONE_EABI_GCC_COMPILER_EXE} )
elseif ( EXISTS ${ARM_NONE_EABI_GITHUB_GCC_COMPILER} )
  set( ARM_NONE_EABI_GCC_COMPILER ${ARM_NONE_EABI_GITHUB_GCC_COMPILER} )
elseif ( EXISTS ${ARM_NONE_EABI_GITHUB_GCC_COMPILER_EXE} )
  set( ARM_NONE_EABI_GCC_COMPILER ${ARM_NONE_EABI_GITHUB_GCC_COMPILER_EXE} )
else()
  message(
    FATAL_ERROR "ARM-NONE-EABI GCC not found.
    ${ARM_NONE_EABI_GITHUB_GCC_COMPILER} ${ARM_NONE_EABI_GCC_COMPILER}
    ${ARM_NONE_EABI_GITHUB_GCC_COMPILER_EXE} ${ARM_NONE_EABI_GCC_COMPILER_EXE}"
    )
endif()

message( "ARM-NONE-EABI GCC found: ${ARM_NONE_EABI_GCC_COMPILER}")

get_filename_component( ARM_NONE_EABI_TOOLCHAIN_BIN_PATH ${ARM_NONE_EABI_GCC_COMPILER} DIRECTORY )
get_filename_component( ARM_NONE_EABI_TOOLCHAIN_BIN_GCC ${ARM_NONE_EABI_GCC_COMPILER} NAME_WE )
get_filename_component( ARM_NONE_EABI_TOOLCHAIN_BIN_EXT ${ARM_NONE_EABI_GCC_COMPILER} EXT )

message( "ARM-NONE-EABI GCC Path: ${ARM_NONE_EABI_TOOLCHAIN_BIN_PATH}" )

STRING( REGEX REPLACE "\-gcc" "-" CROSS_COMPILE ${ARM_NONE_EABI_TOOLCHAIN_BIN_GCC} )
message( "ARM-NONE-EABI Cross Compile: ${CROSS_COMPILE}" )

# The Generic system name is used for embedded targets ( targets without OS ) in
# CMake
set( CMAKE_SYSTEM_NAME          Generic )
set( CMAKE_SYSTEM_PROCESSOR     cortex-m4 )
set( CMAKE_EXECUTABLE_SUFFIX    ".elf" )

# specify the cross compiler. We force the compiler so that CMake doesn't
# attempt to build a simple test program as this will fail without us using
# the -nostartfiles option on the command line
#CMAKE_FORCE_C_COMPILER( "${ARM_NONE_EABI_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}gcc${ARM_NONE_EABI_TOOLCHAIN_BIN_EXT}" GNU )
#CMAKE_FORCE_CXX_COMPILER( "${ARM_NONE_EABI_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}g++${ARM_NONE_EABI_TOOLCHAIN_BIN_EXT}" GNU )
set( CMAKE_ASM_COMPILER {CROSS_COMPILE}gcc )
set( CMAKE_AR ${CROSS_COMPILE}ar )
set( CMAKE_ASM_COMPILER ${CROSS_COMPILE}gcc )
set( CMAKE_C_COMPILER ${CROSS_COMPILE}gcc )
set( CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++ )

# We must set the OBJCOPY setting into cache so that it's available to the
# whole project. Otherwise, this does not get set into the CACHE and therefore
# the build doesn't know what the OBJCOPY filepath is
set( CMAKE_OBJCOPY      ${ARM_NONE_EABI_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}objcopy
     CACHE FILEPATH "The toolchain objcopy command " FORCE )

set( CMAKE_OBJDUMP      ${ARM_NONE_EABI_TOOLCHAIN_BIN_PATH}/${CROSS_COMPILE}objdump
     CACHE FILEPATH "The toolchain objdump command " FORCE )

# Set the common build flags

# Set the CMAKE C flags ( which should also be used by the assembler!
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --param max-inline-insns-single=500 -mlong-calls -ffunction-sections -fdata-sections -fno-exceptions -fsingle-precision-constant -Wall -Wextra -Wundef -Wdouble-promotion -Wno-expansion-to-defined")

set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "" )
set( CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-threadsafe-statics -fno-rtti" CACHE STRING "" )
set( CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "" )

message( "C_FLAGS: " ${CMAKE_C_FLAGS} )
message( "CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )
message( "ASM_FLAGS: " ${CMAKE_ASM_FLAGS} )

set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}" )
message( "LD_FLAGS: " ${CMAKE_EXE_LINKER_FLAGS} )
