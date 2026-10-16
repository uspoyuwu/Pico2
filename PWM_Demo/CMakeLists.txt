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
set(PICO_BOARD pico2 CACHE STRING "Board type")
# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# Generate compile_commands.json for VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# give the project a name (anything you want)
project(PWM_Demo C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(PWM_Demo)

# must match with executable name and source file names
target_sources(PWM_Demo PRIVATE pwm-demo.c)
# Add pico_multicore which is required for multicore functionality
target_link_libraries(PWM_Demo pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pwm hardware_irq hardware_uart)

# Disable USB Serial
pico_enable_stdio_usb(PWM_Demo 0)

# Enable UART Serial
pico_enable_stdio_uart(PWM_Demo 1)
# create map/bin/hex file etc.
# pico_enable_stdio_uart(PWM_Demo 1)  
# must match with executable name
pico_add_extra_outputs(PWM_Demo)