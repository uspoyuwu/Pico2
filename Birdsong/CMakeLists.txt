# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Keypad C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(Keypad)

# must match with executable name and source file names
target_sources(Keypad PRIVATE keypad.c)

# must match with executable name
target_link_libraries(  Keypad
                        PRIVATE
                        pico_stdlib
                        pico_multicore
                        pico_bootsel_via_double_reset
                        hardware_pio
                        hardware_dma
                        hardware_sync
                        hardware_spi
                        hardware_clocks
                        hardware_adc)

# must match with executable name
pico_add_extra_outputs(Keypad)
