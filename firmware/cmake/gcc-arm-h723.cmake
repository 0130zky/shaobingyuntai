set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# ---------------------------------------------------------------------------
# 定位 arm-none-eabi 工具链。
#
# 注意：Homebrew 的 arm-none-eabi-gcc formula 只装了编译器，不带 newlib
# （编译时会报 fatal error: stdint.h: No such file or directory）。
# 真正能用的是官方 Arm GNU Toolchain 或者
# `brew install --cask gcc-arm-embedded`。
# 所以下面每个候选都要额外验证 newlib 头文件确实存在才算数。
#
# 查找顺序：
#   1. cmake 命令行 -DARM_TOOLCHAIN_PATH=<dir>
#   2. 环境变量 $ARM_TOOLCHAIN_PATH
#   3. 常见安装位置（官方 pkg 装到 /Applications/ArmGNUToolchain/，
#      手动解压一般放 ~/.local/share/arm-gnu-toolchain/）
#   4. PATH 上的 arm-none-eabi-gcc（同样要通过 newlib 检查）
#
# <dir> 可以是三种形式之一：
#   工具链根目录          <dir>/arm-none-eabi/bin/arm-none-eabi-gcc
#   工具链根目录（新版）  <dir>/bin/arm-none-eabi-gcc
#   直接放可执行文件的 bin 目录  <dir>/arm-none-eabi-gcc
# ---------------------------------------------------------------------------

# 把一个候选目录规范化成 bin 目录；同时校验 newlib 是否存在。
# 通过则把结果写入 ${_out_var}，否则保持为空。
function(_dmz_find_toolchain_bin _dir _out_var)
    set(${_out_var} "" PARENT_SCOPE)
    if(NOT _dir OR NOT IS_DIRECTORY "${_dir}")
        return()
    endif()

    set(_candidates
        "${_dir}"                          # 已经是 bin 目录
        "${_dir}/arm-none-eabi/bin"        # 官方包解压后的根
        "${_dir}/bin"                      # 部分发行版的布局
    )

    foreach(_bin IN LISTS _candidates)
        if(NOT EXISTS "${_bin}/arm-none-eabi-gcc")
            continue()
        endif()
        # newlib 头文件必须存在，否则是"半残"的工具链，跳过。
        # 官方布局：<root>/arm-none-eabi/bin -> <root>/arm-none-eabi/arm-none-eabi/include
        if(EXISTS "${_bin}/../arm-none-eabi/include/stdint.h"
           OR EXISTS "${_bin}/../include/stdint.h")
            set(${_out_var} "${_bin}" PARENT_SCOPE)
            return()
        endif()
        message(STATUS "跳过缺少 newlib 的工具链: ${_bin}")
    endforeach()
endfunction()

file(GLOB _arm_roots
    "/Applications/ArmGNUToolchain/*"
    "$ENV{HOME}/.local/share/arm-gnu-toolchain/*"
    "$ENV{HOME}/.local/arm-toolchain/*"
    "/opt/arm-gnu-toolchain/*"
)

set(_arm_hints
    "${ARM_TOOLCHAIN_PATH}"
    "$ENV{ARM_TOOLCHAIN_PATH}"
    ${_arm_roots}
)

set(_arm_bin "")
foreach(_dir IN LISTS _arm_hints)
    _dmz_find_toolchain_bin("${_dir}" _arm_bin)
    if(_arm_bin)
        break()
    endif()
endforeach()

# 最后退回 PATH 上的编译器
if(NOT _arm_bin)
    find_program(_arm_gcc arm-none-eabi-gcc)
    if(_arm_gcc)
        get_filename_component(_arm_dir "${_arm_gcc}" DIRECTORY)
        _dmz_find_toolchain_bin("${_arm_dir}" _arm_bin)
    endif()
endif()

if(NOT _arm_bin)
    message(FATAL_ERROR
        "找不到可用的 arm-none-eabi 工具链（需要带 newlib）。\n"
        "安装方式二选一：\n"
        "    brew install --cask gcc-arm-embedded\n"
        "    # 或从 developer.arm.com 下载后解压到 ~/.local/share/arm-gnu-toolchain/\n"
        "已有安装也可以显式指定：\n"
        "    cmake --preset Debug -DARM_TOOLCHAIN_PATH=/path/to/toolchain/arm-none-eabi")
endif()

# 工具链文件会被处理两次，这条信息只打一遍免得看着像重复配置
if(NOT _DMZ_TOOLCHAIN_MSG_SHOWN)
    set(_DMZ_TOOLCHAIN_MSG_SHOWN TRUE CACHE INTERNAL "toolchain banner printed")
    message(STATUS "ARM toolchain: ${_arm_bin}")
endif()

set(TOOLCHAIN_PREFIX                ${_arm_bin}/arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---------------------------------------------------------------------------
# 编译/链接参数
#
# 重要：CMake 会把工具链文件处理**两次**（先探测编译器 ABI，再正式配置），
# 而下面这些都是 "${CMAKE_C_FLAGS} ..." 形式的**追加**。不做保护的话
# 每次都会再叠一遍，实测编译行里会出现两组 -mcpu/-mfpu/-mfloat-abi。
# 所以整块用变量挡住，只生效一次。
# ---------------------------------------------------------------------------
if(_DMZ_TOOLCHAIN_FLAGS_SET)
    return()
endif()
set(_DMZ_TOOLCHAIN_FLAGS_SET TRUE)

# STM32H723VGT6: Cortex-M7，双精度 FPU (FPv5-D16)，硬件浮点
set(_dmz_target_flags "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${_dmz_target_flags} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")

# 优化等级只在这里定义一处。Debug 用 -Og 而不是 -O0：
# -Og 仍然可以正常单步和看变量，但代码明显比 -O0 快，
# 1kHz 控制任务在 -O0 下容易跑不满。
set(CMAKE_C_FLAGS_DEBUG   "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")

set(CMAKE_EXE_LINKER_FLAGS "${_dmz_target_flags}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/boards/dm_mc02/STM32H723VGTX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
# binutils >= 2.39 会对 "包含 .text 和 .data 的 LOAD 段" 报 RWX 警告。
# 裸机链接脚本里这很正常（没有 MMU 来区分权限），关掉免得刷屏。
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-warn-rwx-segments")
set(TOOLCHAIN_LINK_LIBRARIES "m")
