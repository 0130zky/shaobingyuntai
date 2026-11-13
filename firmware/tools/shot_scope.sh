#!/bin/sh
# 弹速上位机启动脚本（用工程内的 .venv）
#
#   tools/shot_scope.sh                      打开界面
#   tools/shot_scope.sh --replay x.csv       直接载入记录
#   tools/shot_scope.sh --port /dev/cu.usbmodemXXXX
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
PY="$ROOT/.venv/bin/python"

if [ ! -x "$PY" ]; then
    cat >&2 <<EOF
找不到 venv: $PY

重建方法:
    /opt/homebrew/bin/python3.14 -m venv "$ROOT/.venv"
    "$ROOT/.venv/bin/pip" install pyserial numpy matplotlib

（若 python@3.14 没装 tkinter, 先:  brew install python-tk@3.14）
EOF
    exit 1
fi

LOG="$HERE/shot_scope.log"
printf '\n--- %s  shot_scope.sh 启动 ---\n' "$(date '+%Y-%m-%d %H:%M:%S')" >> "$LOG"
"$PY" "$HERE/shot_scope.py" "$@" 2>&1 | tee -a "$LOG"
status=${PIPESTATUS[0]:-0}
printf -- '--- %s  shot_scope.sh 退出 (状态 %s) ---\n' \
       "$(date '+%Y-%m-%d %H:%M:%S')" "$status" >> "$LOG"
exit "$status"
