# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# select the target board type
set(PICO_BOARD pico CACHE STRING "Board type")

# define firmware option configuration
option(RPIO_ENABLE_HUB75 "Enable Hub75 driver." OFF)
option(RPIO_ENABLE_FB "Enable Framebuffer support." ${RPIO_ENABLE_HUB75})

option(RPIO_ENABLE_SPI "Enable SPI interface." ON)

# setup pico-sdk and rpio project
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.0.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(rpio C CXX ASM)
pico_sdk_init()

add_subdirectory(include)
add_subdirectory(src)