cmake_minimum_required(VERSION 3.25)

include(pico_sdk_import.cmake)

project(mpu6050_i2c C CXX ASM)
#set(PICO_SDK_PATH "/home/surya/pico/pico-sdk")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

if (TARGET tinyusb_device)
    add_executable(mpu6050_i2c
    mpu6050_i2c.c
    WS2812.cpp
            )


# pull in common dependencies and additional i2c hardware support
target_link_libraries(mpu6050_i2c pico_stdlib hardware_i2c hardware_sleep hardware_pio)

pico_generate_pio_header(mpu6050_i2c ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu6050_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(mpu6050_i2c 1)
pico_enable_stdio_uart(mpu6050_i2c 0)

# add url via pico_set_program_url
#example_auto_set_url(mpu6050_i2c)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()