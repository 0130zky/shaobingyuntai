set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# ---------------------------------------------------------------------------
# Locate the arm-none-eabi toolchain.
#
# Search order:
#   1. -DARM_TOOLCHAIN_PATH=<dir> on the cmake command line
#   2. $ARM_TOOLCHAIN_PATH environment variable
#   3. Common install locations for the official Arm GNU Toolchain
#   4. Whatever `arm-none-eabi-gcc` happens to be on PATH
#
# <dir> may be either the directory that holds arm-none-eabi-gcc directly,
# or the toolchain root that contains arm-none-eabi/bin/.
# ---------------------------------------------------------------------------

set(_arm_hints
    "${ARM_TOOLCHAIN_PATH}"
    "$ENV{ARM_TOOLCHAIN_PATH}"
)

# Official Arm GNU Toolchain roots (Homebrew cask `gcc-arm-embedded`, .pkg
# installer, or a manual extract under ~/.local/share).
file(GLOB _arm_roots
    "/Applications/ArmGNUToolchain/*"
    "$ENV{HOME}/.local/share/arm-gnu-toolchain/*"
    "$ENV{HOME}/.local/arm-toolchain/*"
    "/opt/arm-gnu-toolchain/*"
)
list(APPEND _arm_hints ${_arm_roots})

set(_arm_bin "")
foreach(_dir IN LISTS _arm_hints)
    if(_dir AND EXISTS "${_dir}/arm-none-eabi-gcc")
        # <dir> already points at the bin directory
        set(_arm_bin "${_dir}")
        break()
    elseif(_dir AND EXISTS "${_dir}/arm-none-eabi/bin/arm-none-eabi-gcc")
        # <dir> is the toolchain root
        set(_arm_bin "${_dir}/arm-none-eabi/bin")
        break()
    endif()
endforeach()

if(NOT _arm_bin)
    find_program(_arm_gcc arm-none-eabi-gcc)
    if(NOT _arm_gcc)
        message(FATAL_ERROR
            "arm-none-eabi-gcc not found.\n"
            "Install the official Arm GNU Toolchain, e.g.:\n"
            "    brew install --cask gcc-arm-embedded\n"
            "or point CMake at an existing install:\n"
            "    cmake --preset Debug -DARM_TOOLCHAIN_PATH=/path/to/arm-none-eabi/bin")
    endif()
    get_filename_component(_arm_bin "${_arm_gcc}" DIRECTORY)
endif()

message(STATUS "ARM toolchain: ${_arm_bin}")

# arm-none-eabi- must be part of the path environment
set(TOOLCHAIN_PREFIX                ${_arm_bin}/arm-none-eabi-)
if(CMAKE_HOST_WIN32)
    set(_arm_exe_suffix ".exe")
else()
    set(_arm_exe_suffix "")
endif()

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc${_arm_exe_suffix})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++${_arm_exe_suffix})
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++${_arm_exe_suffix})
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy${_arm_exe_suffix})
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size${_arm_exe_suffix})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
if(ROBOT STREQUAL "sentry")
    set(_robot_linker "${CMAKE_SOURCE_DIR}/robots/sentry/STM32F407xx_FLASH.ld")
else()
    set(_robot_linker "${CMAKE_SOURCE_DIR}/STM32F407xx_FLASH.ld")
endif()
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${_robot_linker}\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
