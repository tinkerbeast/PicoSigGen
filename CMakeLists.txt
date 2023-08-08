cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
set(PICO_SDK_PATH "/home/rishin/local/pico-sdk")
include(pico_sdk_import.cmake)
# CMake standard settings
project(pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
# Pico examples check
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()
# Initialize the SDK
pico_sdk_init()



add_compile_options(-Wall -Werror
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_executable(pioexp)
pico_generate_pio_header(pioexp ${CMAKE_CURRENT_LIST_DIR}/src/parallel_out.pio)
target_sources(pioexp PRIVATE src/pio_experiment.c src/shell.c)
target_link_libraries(pioexp PRIVATE
                      pico_stdlib pico_multicore hardware_pio hardware_uart)
#  pico_enable_stdio_usb(pioexp 1)
  pico_enable_stdio_uart(pioexp 1)
pico_add_extra_outputs(pioexp)
