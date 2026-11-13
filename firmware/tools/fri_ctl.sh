#!/bin/sh
# 三摩擦轮上位机启动脚本
#
# 用工程内的 .venv（Python 3.14 + pyserial + Tk 9.1）跑 tools/fri_ctl_gui.py。
# 若 .venv 不存在，按下面提示重建。
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
PY="$ROOT/.venv/bin/python"

if [ ! -x "$PY" ]; then
    cat >&2 <<EOF
找不到 venv: $PY

重建方法:
    /opt/homebrew/bin/python3.14 -m venv "$ROOT/.venv"
    "$ROOT/.venv/bin/pip" install pyserial

（若 python@3.14 没装 tkinter, 先:  brew install python-tk@3.14）
EOF
    exit 1
fi

# 崩溃排查：应用层日志由 Python 自己写进同一个文件；
# 这里再把 stdout/stderr 全量 tee 进去，这样连 C 层的 Fatal Python error /
# 段错误也不会丢（那类错误 Python 的 excepthook 抓不到）。
LOG="$HERE/fri_ctl.log"
printf '\n--- %s  fri_ctl.sh 启动 ---\n' "$(date '+%Y-%m-%d %H:%M:%S')" >> "$LOG"
"$PY" "$HERE/fri_ctl_gui.py" "$@" 2>&1 | tee -a "$LOG"
status=${PIPESTATUS[0]:-0}
printf -- '--- %s  fri_ctl.sh 退出 (状态 %s) ---\n' \
       "$(date '+%Y-%m-%d %H:%M:%S')" "$status" >> "$LOG"
exit "$status"
