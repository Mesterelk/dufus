cmake_minimum_required(VERSION 3.12)

# Set the full path to your ARM GCC toolchain
set(CMAKE_C_COMPILER "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin/arm-none-eabi-g++.exe")
set(CMAKE_ASM_COMPILER "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin/arm-none-eabi-gcc.exe")

include(pico_sdk_import.cmake)

project(pico_cube_project C CXX ASM)
pico_sdk_init()

# ---------------- Executables ----------------
add_executable(battery battery.c)
target_link_libraries(battery pico_stdlib hardware_spi hardware_i2c)
pico_add_extra_outputs(battery)

add_executable(module module.c)
target_link_libraries(module pico_stdlib hardware_spi hardware_i2c)
pico_add_extra_outputs(module)
