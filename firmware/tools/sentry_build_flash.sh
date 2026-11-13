#!/usr/bin/env bash
# 编译并烧录 MC02 / STM32H723 哨兵固件。命令行和 VS Code 面板共用这一份逻辑。
#
# 用法:
#   tools/sentry_build_flash.sh build     只编译
#   tools/sentry_build_flash.sh flash     编译 + 烧录 + 复位
#   tools/sentry_build_flash.sh program   只烧录已存在的固件
#
# 选项:
#   --preset NAME   CMake 预设，默认 Sentry（产物 build/SentryH7/）
#   --speed KHZ     SWD 时钟，默认 100（部署验证时使用的速度）
#   --elf PATH     指定固件文件，默认按预设推导
#   --dry-run      只打印将执行的命令，不编译、不连接硬件
#   -h, --help     显示本帮助
#
# 输出每行带 [build] / [flash] 前缀，VS Code 面板据此显示当前阶段。
# 退出码: 0 成功；1 用法错误；2 编译失败；3 找不到固件；4 找不到工具；5 烧录失败
set -euo pipefail

firmware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$firmware_dir"

preset=Sentry
speed=100
dry_run=0
mode=""
elf=""

usage() {
    awk 'NR>1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        build|flash|program) mode="$1";     shift   ;;
        --preset)            preset="${2:?--preset 需要参数}"; shift 2 ;;
        --speed)             speed="${2:?--speed 需要参数}";   shift 2 ;;
        --elf)               elf="${2:?--elf 需要参数}";       shift 2 ;;
        --dry-run)           dry_run=1;     shift   ;;
        -h|--help)           usage 0 ;;
        *) printf '[flash] 未知参数: %s\n' "$1" >&2; usage 1 ;;
    esac
done
[ -n "$mode" ] || usage 1

case "$preset" in
    Sentry)    build_dir="build/SentryH7"; default_elf="$build_dir/sentry_dm_mc02.elf" ;;
    SentryF407) build_dir="build/SentryF407"; default_elf="$build_dir/sentry_dm_mc02.elf" ;;
    *)         build_dir="build/$preset";  default_elf="$build_dir/robot_template.elf" ;;
esac
[ -n "$elf" ] || elf="$default_elf"

find_openocd() {
    command -v openocd 2>/dev/null && return
    for candidate in /opt/homebrew/bin/openocd /usr/local/bin/openocd /usr/bin/openocd; do
        [ -x "$candidate" ] && { printf '%s\n' "$candidate"; return; }
    done
    return 1
}

run_build() {
    if [ "$dry_run" -eq 1 ]; then
        printf '[build] cmake --preset %s\n' "$preset"
        printf '[build] cmake --build --preset %s\n' "$preset"
        return
    fi
    command -v cmake >/dev/null 2>&1 || { printf '[build] 找不到 cmake\n' >&2; exit 4; }
    printf '[build] cmake --preset %s\n' "$preset"
    cmake --preset "$preset" || { printf '[build] 配置失败\n' >&2; exit 2; }
    printf '[build] cmake --build --preset %s\n' "$preset"
    cmake --build --preset "$preset" || { printf '[build] 编译失败\n' >&2; exit 2; }
    if [ ! -f "$elf" ]; then
        printf '[build] 编译结束但找不到 %s；用 --elf 指定文件\n' "$elf" >&2
        exit 3
    fi
}

run_flash() {
    local openocd
    openocd="$(find_openocd)" || { printf '[flash] 找不到 openocd，请先 brew install openocd\n' >&2; exit 4; }
    if [ ! -f "$elf" ] && [ "$dry_run" -eq 0 ]; then
        printf '[flash] 找不到固件 %s，先编译或用 --elf 指定\n' "$elf" >&2
        exit 3
    fi
    local args=(
        -f interface/cmsis-dap.cfg
        -c "cmsis_dap_backend usb_bulk; transport select swd"
        -f target/stm32h7x.cfg
        -c "adapter speed $speed; stm32h7x.cpu0 configure -event reset-init {adapter speed $speed}"
        -c "program \"$firmware_dir/$elf\" verify reset exit"
    )
    if [ -f "$elf" ]; then
        printf '[flash] %s (%s)\n' "$elf" "$(du -h "$elf" | cut -f1)"
    else
        printf '[flash] %s（不存在）\n' "$elf"
    fi
    if [ "$dry_run" -eq 1 ]; then
        printf '[flash] %s' "$openocd"
        for arg in "${args[@]}"; do
            case "$arg" in
                *" "*) case "$arg" in
                           *"'"*) printf ' "%s"' "$arg" ;;
                           *)     printf " '%s'" "$arg" ;;
                       esac ;;
                *)     printf ' %s' "$arg" ;;
            esac
        done
        printf '\n[flash] dry-run，未连接硬件\n'
        return
    fi
    printf '[flash] openocd 写入并校验，成功后复位运行\n'
    "$openocd" "${args[@]}" 2>&1 || {
        printf '[flash] 烧录失败。检查：板子供电、DAPLink 连接、SWD 线序、是否只有一个探针\n' >&2
        exit 5
    }
    printf '[flash] 烧录完成并已复位\n'
}

case "$mode" in
    build)   run_build ;;
    flash)   run_build; run_flash ;;
    program) run_flash ;;
esac
