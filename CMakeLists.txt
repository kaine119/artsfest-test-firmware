cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

set(PICO_EXTRAS_FETCH_FROM_GIT true)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(artsfest-test C CXX ASM)

pico_sdk_init()

include(FreeRTOS_Kernel_import.cmake)

add_executable(artsfest-test
    main.c
    touch_task.c
    audio_task.c
)

target_include_directories(artsfest-test PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_compile_definitions(artsfest-test PRIVATE
    PICO_AUDIO_I2S_MONO_INPUT=1
)

target_link_libraries(artsfest-test PRIVATE
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    hardware_i2c
    pico_audio_i2s
)

pico_enable_stdio_usb(artsfest-test 1)
pico_enable_stdio_uart(artsfest-test 0)

pico_add_extra_outputs(artsfest-test)