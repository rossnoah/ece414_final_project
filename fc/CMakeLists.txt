# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(flightcontroller C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(flightcontroller
    flightcontroller.cpp
    crsf.cpp
    timer.cpp
    dshot_encoder.cpp
    pico_pio_loader.cpp
    imu.cpp
    barometer.cpp
    hc05.cpp
)

pico_generate_pio_header(flightcontroller ${CMAKE_CURRENT_LIST_DIR}/dshot_encoder.pio)

# pull in common dependencies and additional uart hardware support
target_link_libraries(flightcontroller pico_stdlib hardware_uart pico_bootsel_via_double_reset hardware_pwm hardware_pio hardware_sync hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(flightcontroller)

# add url via pico_set_program_url

# set stdio via usb
pico_enable_stdio_usb(flightcontroller 1)
pico_enable_stdio_uart(flightcontroller 0)