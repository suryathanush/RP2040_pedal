# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/surya/pico/pico-sdk/tools/pioasm"
  "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pioasm"
  "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/surya/Upwork/amit foxtini/rp2040_pedal/mpu6050_i2c/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
