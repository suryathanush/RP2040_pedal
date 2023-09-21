cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)


project(pico_SDK_proj C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

#pico_set_program_name(pico_SDK_project "pico_SDK_project")
#pico_set_program_version(pico_SDK_project "1.0")

add_executable(pico_SDK_proj
        pico_SDK_proj.cpp
        WS2812.cpp
        )

# pull in common dependencies
target_link_libraries(pico_SDK_proj pico_stdlib)

# Add any user requested libraries
target_link_libraries(pico_SDK_proj
        hardware_pio
)

pico_generate_pio_header(pico_SDK_proj ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)

pico_enable_stdio_uart(pico_SDK_proj 0)
pico_enable_stdio_usb(pico_SDK_proj 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_SDK_proj)

