cmake_minimum_required(VERSION 3.22)
project(mc02_can_probe C ASM)
set(CMAKE_C_STANDARD 11)
get_filename_component(FW ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
set(B ${FW}/boards/dm_mc02)
file(GLOB hal ${B}/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
list(FILTER hal EXCLUDE REGEX "template")
option(ID_EDITOR "Explicit mailbox for authorized ESC ID 1->2 change" OFF)
set(probe_source probe.c)
if(ID_EDITOR)
 set(probe_source id_editor.c)
endif()
add_executable(mc02_can_probe main.c ${probe_source} ${hal}
 ${B}/Core/Src/gpio.c ${B}/Core/Src/fdcan.c
 ${B}/Core/Src/stm32h7xx_hal_msp.c ${B}/Core/Src/stm32h7xx_hal_timebase_tim.c
 ${B}/Core/Src/system_stm32h7xx.c ${B}/Core/Src/startup_stm32h723xx.s)
target_compile_definitions(mc02_can_probe PRIVATE USE_HAL_DRIVER STM32H723xx)
target_include_directories(mc02_can_probe PRIVATE ${B}/Core/Inc
 ${B}/Drivers/STM32H7xx_HAL_Driver/Inc ${B}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
 ${B}/Drivers/CMSIS/Device/ST/STM32H7xx/Include ${B}/Drivers/CMSIS/Include)
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -T ${CMAKE_CURRENT_SOURCE_DIR}/RAM.ld --specs=nano.specs -Wl,--gc-sections -Wl,--no-warn-rwx-segments")
target_link_libraries(mc02_can_probe PRIVATE m)

option(PROBE_DISABLE_READ "Operator-approved disable-only feedback scan at 1Mbps" OFF)
if(PROBE_DISABLE_READ)
 target_compile_definitions(mc02_can_probe PRIVATE PROBE_DISABLE_READ=1)
endif()

if(ID_EDITOR)
 set(ID_EDITOR_FEEDBACK_ID "0x11" CACHE STRING "Expected physical motor feedback ID")
 target_compile_definitions(mc02_can_probe PRIVATE ID_EDITOR_FEEDBACK_ID=${ID_EDITOR_FEEDBACK_ID})
endif()
