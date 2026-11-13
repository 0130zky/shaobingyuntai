set(B ${CMAKE_CURRENT_SOURCE_DIR}/boards/dm_mc02)
add_executable(${CMAKE_PROJECT_NAME})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_HAL_DRIVER STM32H723xx SENTRY_STM32H7)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${B}/Core/Inc ${B}/USB_DEVICE/App ${B}/USB_DEVICE/Target
    robots/sentry modules/folding_gimbal Application Components/Devices
    ${B}/Drivers/STM32H7xx_HAL_Driver/Inc ${B}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${B}/Drivers/CMSIS/Device/ST/STM32H7xx/Include ${B}/Drivers/CMSIS/Include
    ${B}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${B}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${B}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
    ${B}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${B}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)
file(GLOB core CONFIGURE_DEPENDS ${B}/Core/Src/*.c ${B}/Core/Src/*.s)
file(GLOB hal ${B}/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
list(FILTER hal EXCLUDE REGEX "template")
file(GLOB rtos ${B}/Middlewares/Third_Party/FreeRTOS/Source/*.c)
file(GLOB usb ${B}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c
    ${B}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c
    ${B}/USB_DEVICE/App/*.c ${B}/USB_DEVICE/Target/*.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${core} ${hal} ${rtos} ${usb}
    ${B}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${B}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c
    ${B}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    modules/folding_gimbal/sentry_remote.c Components/Devices/app_DM4310.c
    modules/folding_gimbal/sentry.c modules/folding_gimbal/storage.c modules/folding_gimbal/z_linkage.c modules/folding_gimbal/motion.c)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE m)
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS ${B}/STM32H723VGTX_FLASH.ld)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
