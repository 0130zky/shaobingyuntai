#!/usr/bin/env bash
#
# 简易烧录脚本 —— OpenOCD + DAPLink (CMSIS-DAP)
#
# 用法:
#   ./tools/flash.sh                  烧录最新编译出的固件
#   ./tools/flash.sh -b               先编译再烧录
#   ./tools/flash.sh -p DM4310       选择构建配置（默认 Shooter）
#   ./tools/flash.sh -f foo.hex       烧录指定文件 (.elf / .hex / .bin)
#   ./tools/flash.sh -e               整片擦除
#   ./tools/flash.sh -r               仅复位运行
#   ./tools/flash.sh -s <序列号>      指定 DAPLink 探针 (接多个时用)
#   ./tools/flash.sh -n               只打印命令，不真的连接
#   ./tools/flash.sh -h               帮助
#
set -euo pipefail

# ---------------------------------------------------------------------------
# 默认配置
# ---------------------------------------------------------------------------
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRMWARE_NAME="robot_template"
TARGET_CFG="target/stm32f4x.cfg"
INTERFACE_CFG="interface/cmsis-dap.cfg"
ADAPTER_SERIAL=""
BUILD_FIRST=0
CMAKE_PRESET="Shooter"
FILE=""
MODE="flash"          # flash | erase | reset
DRY_RUN=0
BIN_ADDR="0x08000000" # .bin 文件的烧录起始地址
VERIFY=1

# ---------------------------------------------------------------------------
# 输出helper
# ---------------------------------------------------------------------------
if [ -t 1 ]; then
    C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'; C_DIM=$'\033[2m'; C_OFF=$'\033[0m'
else
    C_RED=""; C_GRN=""; C_YEL=""; C_DIM=""; C_OFF=""
fi

info()  { printf '%s[*]%s %s\n' "$C_DIM" "$C_OFF" "$*"; }
ok()    { printf '%s[+]%s %s\n' "$C_GRN" "$C_OFF" "$*"; }
warn()  { printf '%s[!]%s %s\n' "$C_YEL" "$C_OFF" "$*" >&2; }
die()   { printf '%s[x]%s %s\n' "$C_RED" "$C_OFF" "$*" >&2; exit 1; }

usage() {
    # 打印开头的注释块（跳过 shebang，遇到第一行非注释就停）
    awk 'NR>1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
    exit 0
}

# ---------------------------------------------------------------------------
# 解析参数
# ---------------------------------------------------------------------------
while [ $# -gt 0 ]; do
    case "$1" in
        -f|--file)      FILE="${2:-}";        shift 2 ;;
        -b|--build)     BUILD_FIRST=1;        shift   ;;
        -p|--preset)    CMAKE_PRESET="${2:-}";shift 2 ;;
        -e|--erase)     MODE="erase";         shift   ;;
        -r|--reset)     MODE="reset";         shift   ;;
        -s|--serial)    ADAPTER_SERIAL="${2:-}"; shift 2 ;;
        -a|--adapter)   INTERFACE_CFG="${2:-}"; shift 2 ;;
        -t|--target)    TARGET_CFG="${2:-}";  shift 2 ;;
        -n|--dry-run)   DRY_RUN=1;            shift   ;;
        --no-verify)    VERIFY=0;             shift   ;;
        -h|--help)      usage ;;
        *) die "未知参数: $1  (用 -h 查看帮助)" ;;
    esac
done
case "$CMAKE_PRESET" in
    Shooter|Bench|Calibration|Debug|Release|DM4310|DM4310Calib) ;;
    *) die "不支持的配置：$CMAKE_PRESET；旧 RM3508 拨盘版本已删除" ;;
esac

# ---------------------------------------------------------------------------
# 找 openocd
# ---------------------------------------------------------------------------
find_openocd() {
    if command -v openocd >/dev/null 2>&1; then
        command -v openocd
        return
    fi
    for p in /opt/homebrew/bin/openocd /usr/local/bin/openocd /usr/bin/openocd; do
        [ -x "$p" ] && { echo "$p"; return; }
    done
    return 1
}

if [ "$DRY_RUN" -eq 0 ]; then
    OPENOCD="$(find_openocd)" || die "找不到 openocd。请先安装: brew install openocd"
else
    OPENOCD="$(find_openocd || echo openocd)"
fi

# ---------------------------------------------------------------------------
# 可选: 先编译
# ---------------------------------------------------------------------------
if [ "$BUILD_FIRST" -eq 1 ]; then
    info "编译 (preset: $CMAKE_PRESET) ..."
    cmake --preset "$CMAKE_PRESET" -S "$PROJECT_ROOT" >/dev/null
    cmake --build "$PROJECT_ROOT/build/$CMAKE_PRESET"
    ok "编译完成"
fi

# ---------------------------------------------------------------------------
# 找固件文件
# ---------------------------------------------------------------------------
find_firmware() {
    # 仅从指定 preset 找固件，避免误烧旧电机版本或零力矩标定版本。
    local ext
    for ext in elf hex; do
        local firmware="$PROJECT_ROOT/build/$CMAKE_PRESET/${FIRMWARE_NAME}.${ext}"
        [ -f "$firmware" ] && { echo "$firmware"; return 0; }
    done
    return 1
}

if [ "$MODE" = "flash" ]; then
    if [ -z "$FILE" ]; then
        FILE="$(find_firmware)" || die "在 build/ 下找不到指定 preset ($CMAKE_PRESET) 的 ${FIRMWARE_NAME}.elf/.hex，先编译或用 -f 指定文件"
    fi
    [ -e "$FILE" ] || die "文件不存在: $FILE"
    FILE="$(cd "$(dirname "$FILE")" && pwd)/$(basename "$FILE")"
fi

# ---------------------------------------------------------------------------
# 组装 openocd 参数
# ---------------------------------------------------------------------------
OC_ARGS=(-f "$INTERFACE_CFG")
[ -n "$ADAPTER_SERIAL" ] && OC_ARGS+=(-c "adapter serial $ADAPTER_SERIAL")
# cmsis-dap.cfg 不指定传输方式，这里显式选 SWD（DAPLink 默认可能是 JTAG）
OC_ARGS+=(-c "transport select swd")
OC_ARGS+=(-f "$TARGET_CFG")
OC_ARGS+=(-c "adapter speed 4000")

case "$MODE" in
    flash)
        VERIFY_KW=""
        [ "$VERIFY" -eq 1 ] && VERIFY_KW="verify"
        if [ "${FILE##*.}" = "bin" ]; then
            # .bin 是裸镜像，必须显式给起始地址
            ACTION="program \"$FILE\" $BIN_ADDR $VERIFY_KW reset exit"
        else
            ACTION="program \"$FILE\" $VERIFY_KW reset exit"
        fi
        ;;
    erase)
        ACTION="init; reset halt; stm32f4x mass_erase 0; reset run; exit"
        ;;
    reset)
        ACTION="init; reset run; exit"
        ;;
esac

# ---------------------------------------------------------------------------
# 干活
# ---------------------------------------------------------------------------
printf '\n'
info "openocd : $OPENOCD"
info "interface: $INTERFACE_CFG"
info "target   : $TARGET_CFG"
case "$MODE" in
    flash) info "firmware : $FILE ($(du -h "$FILE" | cut -f1))" ;;
    erase) info "动作     : 整片擦除" ;;
    reset) info "动作     : 复位运行" ;;
esac
printf '\n'

CMD=("$OPENOCD" "${OC_ARGS[@]}" -c "$ACTION")

if [ "$DRY_RUN" -eq 1 ]; then
    warn "dry-run，未连接硬件。将执行:"
    printf '  %q' "${CMD[@]}"; printf '\n'
    exit 0
fi

set +e
"${CMD[@]}"
rc=$?
set -e

if [ $rc -ne 0 ]; then
    printf '\n'
    die "openocd 失败 (退出码 $rc)。常见原因:
   - 板子没上电 / USB 没插好
   - DAPLink 没被识别 (macOS: system_profiler SPUSBDataType | grep -i -A5 daplink)
   - SWD 线序接错 (SWCLK / SWDIO / GND / 3V3)
   - 接了多个探针，用 -s <序列号> 指定"
fi

printf '\n'
case "$MODE" in
    flash) ok "烧录完成并已复位运行 ✔" ;;
    erase) ok "擦除完成 ✔" ;;
    reset) ok "已复位运行 ✔" ;;
esac
