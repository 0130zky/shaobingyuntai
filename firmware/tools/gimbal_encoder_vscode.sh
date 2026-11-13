#!/bin/sh
# 在当前VS Code窗口内打开已安装的云台编码器面板。
set -eu
if ! command -v code >/dev/null 2>&1; then
    echo '找不到code命令，请在VS Code中安装shell命令code。' >&2
    exit 1
fi
exec code --open-url 'vscode://local-robot-template.robot-template-gimbal-encoder/show'
