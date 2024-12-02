cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(final_project)
pico_sdk_init()
add_executable(final_project
)

pico_generate_pio_header(final_project ${CMAKE_CURRENT_LIST_DIR}/SPIPIO.pio)
# Link with the Pico SDK librarie
# pull in common dependencies
target_link_libraries(final_project
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_pio
    hardware_irq
    hardware_pwm
    hardware_i2c
    pico_bootsel_via_double_reset
    # PIO_input_capture_2a
   #ic
    # ic_pio
)

target_sources(final_project PRIVATE
    main.c            
    imu.c
)
# Specify the location of the build directory
# create map/bin/hex file etc.
pico_add_extra_outputs(final_project)
