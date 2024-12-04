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

include(pico_sdk_import.cmake)

project(mpu6050_i2c C CXX ASM)

pico_sdk_init()

add_executable(mpu6050_i2c
    mpu6050_i2c.c
    imu.c
    timer.c
)

target_link_libraries(mpu6050_i2c pico_stdlib hardware_i2c pico_bootsel_via_double_reset
)

pico_add_extra_outputs(mpu6050_i2c)

# set stdio via usb
pico_enable_stdio_usb(mpu6050_i2c 1)
pico_enable_stdio_uart(mpu6050_i2c 0)