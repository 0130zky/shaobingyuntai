#!/bin/sh
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
exec python3 "$HERE/shot_monitor.py" "$@"
