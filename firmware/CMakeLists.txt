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
set(CMAKE_PROJECT_NAME robot_template)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
if(BOARD STREQUAL "DM_MC02")
    if(NOT ROBOT STREQUAL "sentry")
        message(FATAL_ERROR "DM_MC02 supports ROBOT=sentry only")
    endif()
    set(ROBOT sentry CACHE STRING "Robot assembly" FORCE)
    set(CMAKE_PROJECT_NAME sentry_dm_mc02)
    project(${CMAKE_PROJECT_NAME} C ASM)
    include(boards/dm_mc02/build.cmake)
    return()
endif()

project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
# NOTE: must exist before cmake/stm32cubemx is added, because that file
#       attaches the CubeMX generated sources to this target.
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(ROBOT "example" CACHE STRING "Robot assembly: example | minimal | sentry")
set_property(CACHE ROBOT PROPERTY STRINGS example minimal sentry)
if(NOT ROBOT STREQUAL "example" AND NOT ROBOT STREQUAL "minimal" AND NOT ROBOT STREQUAL "sentry")
    message(FATAL_ERROR "Unknown ROBOT: ${ROBOT}; add its assembly explicitly")
endif()
target_sources(${CMAKE_PROJECT_NAME} PRIVATE robots/${ROBOT}/robot.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE robots/${ROBOT})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOT_HAS_REFEREE=$<STREQUAL:${ROBOT},example>)

# ---------------------------------------------------------------------------
# User sources
# ---------------------------------------------------------------------------
set(USER_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/app_tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/PID_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/RC_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/app_can_router.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/referee_task.c
)

set(USER_Bsp_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_dwt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_wbus.c
)

set(USER_Components_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Devices/app_DM4310.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Devices/app_RM_Motor.c
)

set(USER_Referee_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Referee/CRC8_CRC16.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Referee/fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Referee/referee.c
)

set(USER_Sys_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Sys/sys.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_Application_Src}
    ${USER_Bsp_Src}
    ${USER_Referee_Src}
    ${USER_Sys_Src}
    Components/Devices/app_et07.c
)

if(ROBOT STREQUAL "example")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_Components_Src}
        Application/booster_task.c Application/module_task.c
        robots/example/remote_mapping.c)
# Fixed hardware; reject old cached motor selections explicitly.
if(DEFINED DIAL_MOTOR AND NOT DIAL_MOTOR STREQUAL "DM4310")
    message(FATAL_ERROR "RM3508 dial removed; use a fresh Shooter build directory")
endif()
set(FIRMWARE_PROFILE "Shooter" CACHE STRING "Shooter | Bench | Calibration")
set_property(CACHE FIRMWARE_PROFILE PROPERTY STRINGS Shooter Bench Calibration)
if(FIRMWARE_PROFILE STREQUAL "Shooter")
    set(BOOSTER_RC_ONLY ON)
    set(DIAL_DM_CALIB OFF)
elseif(FIRMWARE_PROFILE STREQUAL "Bench")
    set(BOOSTER_RC_ONLY OFF)
    set(DIAL_DM_CALIB OFF)
elseif(FIRMWARE_PROFILE STREQUAL "Calibration")
    set(BOOSTER_RC_ONLY OFF)
    set(DIAL_DM_CALIB ON)
else()
    message(FATAL_ERROR "Unknown FIRMWARE_PROFILE: ${FIRMWARE_PROFILE}")
endif()
# Control math has no HAL/RTOS dependency. Shooter internals stay target-private.
add_library(control_algorithms OBJECT Components/Controller/controller.c)
target_include_directories(control_algorithms PRIVATE Components/Controller)
add_library(shooter_core OBJECT
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter/shooter_command.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter/friction_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter/dial_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter/shooter_telemetry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter/shooter_feedback.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter/shooter_time.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter/shooter_status.c
    Components/Devices/dial_port_dm.c)
target_link_libraries(shooter_core PRIVATE stm32cubemx)
target_include_directories(shooter_core PRIVATE
    Application modules/shooter Bsp Components/Devices Components/Controller robots/example)
target_compile_definitions(shooter_core PRIVATE
    BOOSTER_RC_ONLY=$<BOOL:${BOOSTER_RC_ONLY}>
    DIAL_DM_CALIB_FREE=$<BOOL:${DIAL_DM_CALIB}>)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    BOOSTER_RC_ONLY=$<BOOL:${BOOSTER_RC_ONLY}>
    DIAL_DM_CALIB_FREE=$<BOOL:${DIAL_DM_CALIB}>)

# 云台两轴 DM4310（MIT 位置模式），与发射模块独立的 CAN1 ID 空间。
add_library(gimbal_core OBJECT
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/gimbal/gimbal_control.c)
target_link_libraries(gimbal_core PRIVATE stm32cubemx)
target_include_directories(gimbal_core PRIVATE
    Application modules/gimbal modules/shooter Bsp Components/Devices Components/Controller robots/example)

message(STATUS "Four-motor firmware profile: ${FIRMWARE_PROFILE}")

    target_link_libraries(${CMAKE_PROJECT_NAME} shooter_core gimbal_core)
elseif(ROBOT STREQUAL "sentry")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOT_TASK_STACK_WORDS=512 SENTRY_REMOTE_LEGACY)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Components/Devices/app_DM4310.c
        modules/folding_gimbal/sentry.c
        modules/folding_gimbal/z_linkage.c modules/folding_gimbal/motion.c
        modules/folding_gimbal/storage.c
        modules/folding_gimbal/sentry_remote.c robots/sentry/remote_legacy.c)
    target_include_directories(${CMAKE_PROJECT_NAME} BEFORE PRIVATE
        robots/sentry modules/folding_gimbal)
    add_library(control_algorithms OBJECT Components/Controller/controller.c)
    target_include_directories(control_algorithms PRIVATE Components/Controller)
else()
    add_library(control_algorithms OBJECT Components/Controller/controller.c)
    target_include_directories(control_algorithms PRIVATE Components/Controller)
endif()

# ---------------------------------------------------------------------------
# Link directories setup
# ---------------------------------------------------------------------------
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ---------------------------------------------------------------------------
# Add include paths
# ---------------------------------------------------------------------------
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Application
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shooter
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/gimbal
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/Components
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Controller
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/Devices
    ${CMAKE_CURRENT_SOURCE_DIR}/Referee
    ${CMAKE_CURRENT_SOURCE_DIR}/Sys
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ---------------------------------------------------------------------------
# Add linked libraries
# ---------------------------------------------------------------------------
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    control_algorithms

    # Add user defined libraries
)

# ---------------------------------------------------------------------------
# Post-build artifacts: .hex / .bin + memory usage
# ---------------------------------------------------------------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex / ${CMAKE_PROJECT_NAME}.bin"
)

# ---------------------------------------------------------------------------
# Flash / erase targets (DAPLink over CMSIS-DAP, driven by OpenOCD)
# ---------------------------------------------------------------------------
# Equivalent to running tools/flash.sh from the project root.
find_program(OPENOCD_EXECUTABLE openocd)

if(OPENOCD_EXECUTABLE)
    set(OPENOCD_INTERFACE "interface/cmsis-dap.cfg" CACHE STRING
        "OpenOCD interface config (default: DAPLink / CMSIS-DAP)")
    set(OPENOCD_TARGET "target/stm32f4x.cfg" CACHE STRING
        "OpenOCD target config")

    # cmsis-dap.cfg does not pin a transport; select SWD explicitly.
    set(OPENOCD_COMMON
        -f ${OPENOCD_INTERFACE}
        -c "transport select swd"
        -f ${OPENOCD_TARGET}
    )

    add_custom_target(flash
        COMMAND ${OPENOCD_EXECUTABLE}
                ${OPENOCD_COMMON}
                -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} via DAPLink (CMSIS-DAP)"
        USES_TERMINAL
    )

    add_custom_target(erase
        COMMAND ${OPENOCD_EXECUTABLE}
                ${OPENOCD_COMMON}
                -c "init; reset halt; stm32f4x mass_erase 0; reset run; exit"
        COMMENT "Mass-erasing target flash via DAPLink (CMSIS-DAP)"
        USES_TERMINAL
    )

    add_custom_target(reset
        COMMAND ${OPENOCD_EXECUTABLE}
                ${OPENOCD_COMMON}
                -c "init; reset run; exit"
        COMMENT "Resetting target via DAPLink (CMSIS-DAP)"
        USES_TERMINAL
    )
else()
    message(STATUS "openocd not found - 'flash'/'erase'/'reset' targets disabled")
endif()
