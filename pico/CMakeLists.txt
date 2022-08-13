cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(cylinder_ros2_pico_micro_ros C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories($ENV{HOME}/pico/micro_ros_raspberrypi_pico_sdk/libmicroros)
add_executable(cylinder_ros2_pico_micro_ros
    cylinder_ros2_pico_micro_ros.c
    pico_uart_transport.c
)
target_link_libraries(cylinder_ros2_pico_micro_ros
    pico_stdlib
    microros
)

target_include_directories(cylinder_ros2_pico_micro_ros PUBLIC
    $ENV{HOME}/pico/micro_ros_raspberrypi_pico_sdk/libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(cylinder_ros2_pico_micro_ros 1)
pico_enable_stdio_uart(cylinder_ros2_pico_micro_ros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(cylinder_ros2_pico_micro_ros)
