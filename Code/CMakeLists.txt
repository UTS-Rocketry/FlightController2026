cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Code)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/App/AppDrivers/Sensors/BMP388.c
    Core/App/AppDrivers/Sensors/h3lis331dl_hal.c
    Core/App/AppDrivers/Sensors/h3lis331dl_reg.c
    Core/App/AppDrivers/Sensors/lsm6dsox_hal.c
    Core/App/AppDrivers/Sensors/lsm6dsox_reg.c
    Core/App/AppDrivers/CAN/CAN.c
    Core/App/AppDrivers/GPS/GPS.c
    Core/App/AppDrivers/LoRa/LoRa.c
    Core/App/AppDrivers/Memory/W25Q128.c
    Core/App/AppDrivers/Memory/fatfs_sd.c
    Core/App/AppDrivers/USB/USBC.c
    Core/App/ApplicationLayer/ApplicationHALS/flight_sensors.c
    Core/App/ApplicationLayer/ApplicationHALS/Lora_App.c
    Core/App/Outputs/DataLogger/logger.c
    Core/App/Outputs/LED&buzzer/indicators.c
    Core/App/Outputs/Telemetry/telemetry.c
    Core/App/ApplicationLayer/ApplicationHALS/crc16.c
    Core/App/ApplicationLayer/ApplicationHALS/W25Q128_HAL.c
    Core/App/ApplicationLayer/ApplicationHALS/pyro.c
    Core/App/ApplicationLayer/ApplicationHALS/kalman.c
    Core/App/ApplicationLayer/FlightStateMachine/flight_state.c
    Core/App/ApplicationLayer/ApplicationHALS/packets.c
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/App/AppDrivers/Sensors
    Core/App/AppDrivers/CAN
    Core/App/AppDrivers/GPS
    Core/App/AppDrivers/LoRa
    Core/App/AppDrivers/Memory
    Core/App/AppDrivers/USB
    Core/App/ApplicationLayer/ApplicationHALS/
    Core/App/Outputs/DataLogger
    Core/App/Outputs/LED&buzzer
    Core/App/Outputs/Telemetry
    Core/App/ApplicationLayer/FlightStateMachine/


)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_target(flash
    COMMAND st-flash write ${CMAKE_PROJECT_NAME}.elf 0x08000000
    DEPENDS ${CMAKE_PROJECT_NAME}
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
