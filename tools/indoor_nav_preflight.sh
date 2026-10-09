#!/usr/bin/env bash
set -eo pipefail
TASK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$TASK_WS/devel/setup.bash"
exec timeout 60 python3 "$TASK_WS/tools/indoor_nav_check.py" "$@"
