cmake_minimum_required(VERSION 3.22)
project(dt7_monitor C ASM)
set(CMAKE_C_STANDARD 11)
get_filename_component(FW ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
set(B ${FW}/boards/dm_mc02)
file(GLOB hal ${B}/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
list(FILTER hal EXCLUDE REGEX "template")
add_executable(dt7_monitor main.c ${hal} ${B}/Core/Src/board.c ${FW}/modules/folding_gimbal/sentry_remote.c
 ${B}/Core/Src/gpio.c ${B}/Core/Src/fdcan.c
 ${B}/Core/Src/stm32h7xx_hal_msp.c ${B}/Core/Src/stm32h7xx_hal_timebase_tim.c
 ${B}/Core/Src/system_stm32h7xx.c ${B}/Core/Src/startup_stm32h723xx.s)
target_compile_definitions(dt7_monitor PRIVATE USE_HAL_DRIVER STM32H723xx)
target_include_directories(dt7_monitor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/shim ${FW}/modules/folding_gimbal ${B}/Core/Inc
 ${B}/Drivers/STM32H7xx_HAL_Driver/Inc ${B}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
 ${B}/Drivers/CMSIS/Device/ST/STM32H7xx/Include ${B}/Drivers/CMSIS/Include)
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -T ${CMAKE_CURRENT_SOURCE_DIR}/../can_probe/RAM.ld --specs=nano.specs -Wl,--gc-sections -Wl,--no-warn-rwx-segments")
target_link_libraries(dt7_monitor PRIVATE m)

