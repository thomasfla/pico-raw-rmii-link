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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico-rmii-raw-nowip C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico-rmii-raw-nowip
    pico-rmii-raw-nowip.c
    src/PicoRawRMII_Driver.c
    src/PicoRawRMII.cpp
)

pico_set_program_name(pico-rmii-raw-nowip "pico-rmii-raw-nowip")
pico_set_program_version(pico-rmii-raw-nowip "0.1")

# Generate PIO header
#pico_generate_pio_header(pico-rmii-raw-nowip ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(pico-rmii-raw-nowip ${CMAKE_CURRENT_LIST_DIR}/rmii_ethernet_phy_rx.pio)
pico_generate_pio_header(pico-rmii-raw-nowip  ${CMAKE_CURRENT_LIST_DIR}/rmii_ethernet_phy_tx.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico-rmii-raw-nowip 0)
pico_enable_stdio_usb(pico-rmii-raw-nowip 1)

# Add the standard library to the build
target_link_libraries(pico-rmii-raw-nowip
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_pwm
    pico_unique_id
)

# Add the standard include files to the build
target_include_directories(pico-rmii-raw-nowip PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Add any user requested libraries
target_link_libraries(pico-rmii-raw-nowip 
        hardware_pio
        )

pico_add_extra_outputs(pico-rmii-raw-nowip)

# Enable running with flash at higher system clock frequencies
pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(pico-rmii-raw-nowip slower_boot2)
