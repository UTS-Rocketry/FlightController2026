cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The application carries small bindings for the ODIN board peripherals.
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(odin_flight_zephyr C)

target_sources(app PRIVATE
    Core/Src/main.c
    Core/App/ApplicationLayer/ApplicationHALS/flight_sensors.c
    Core/App/ApplicationLayer/ApplicationHALS/pyro.c
    Core/App/AppDrivers/GPS/GPS.c
    Core/App/ApplicationLayer/ApplicationHALS/Lora_App.c
    Core/App/ApplicationLayer/ApplicationHALS/W25Q128_HAL.c
    Core/App/AppDrivers/CAN/CAN.c
    Core/App/ApplicationLayer/ApplicationHALS/kalman.c
    Core/App/ApplicationLayer/ApplicationHALS/crc16.c
    Core/App/ApplicationLayer/ApplicationHALS/packets.c
    Core/App/ApplicationLayer/FlightStateMachine/flight_state.c
    Core/App/AppDrivers/Sensors/lsm6dsox_reg.c
    Core/App/AppDrivers/Sensors/h3lis331dl_reg.c
)

# Put the compatibility headers first. Hardware access is implemented through
# Zephyr; these narrow HAL types keep portable estimator/FSM code reusable.
target_include_directories(app BEFORE PRIVATE
    Core/Inc
    Core/App/ApplicationLayer/ApplicationHALS
    Core/App/ApplicationLayer/FlightStateMachine
    Core/App/AppDrivers/Sensors
    Core/App/AppDrivers/GPS
    Core/App/AppDrivers/CAN
)

target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Wshadow
    -Wformat=2
    -Wundef
)
