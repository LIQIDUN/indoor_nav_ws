#!/usr/bin/env bash
set -eo pipefail
TASK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TASK_WS"
source /opt/ros/noetic/setup.bash
source "$TASK_WS/devel/setup.bash"
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
exec "$TASK_WS/tools/indoor_nav_preflight.sh" --smoke
