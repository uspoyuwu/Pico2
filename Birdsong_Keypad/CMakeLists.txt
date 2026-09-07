# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(keypad C CXX ASM)

# Generate compile_commands.json for VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
# Add executable. Default name is the project name, version 0.1

add_executable(	  keypad
    		      keypad.c
        	)

# pull in common dependencies
target_link_libraries(keypad 
    pico_stdlib 
    pico_sync
    hardware_sync
    hardware_clocks
    hardware_vreg
    hardware_clocks
    hardware_adc
    hardware_spi
    pico_bootsel_via_double_reset
    )

pico_enable_stdio_uart(keypad 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(keypad)
