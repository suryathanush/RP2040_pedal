cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(lib/RF24/CMakeLists.txt)

project(pedal_proj C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

if (TARGET tinyusb_device)
    add_executable(pedal_proj
            main.cpp
            defaultPins.h
            )

    # pull in common dependencies
    target_link_libraries(pedal_proj 
        RF24
        pico_stdlib
        hardware_spi
        hardware_gpio
        hardware_i2c
        hardware_sleep
    )

    # enable usb output, disable uart output
    pico_enable_stdio_usb(pedal_proj 1)
    pico_enable_stdio_uart(pedal_proj 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(pedal_proj)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building pedal_proj because TinyUSB submodule is not initialized in the SDK")
endif()
