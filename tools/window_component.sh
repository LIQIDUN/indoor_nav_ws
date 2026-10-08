#!/usr/bin/env bash
# A dedicated desktop terminal runs this wrapper. Records its exact PID and keeps its log visible.
set -u
TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$TOOLS_DIR/.." && pwd)"
name="${1:?usage: window_component.sh COMPONENT}"
RUN_DIR="$WS_ROOT/.sitl_runtime"
mkdir -p "$RUN_DIR/logs"
echo "$$" > "$RUN_DIR/${name}.wrapper.pid"
exec > >(tee -a "$RUN_DIR/logs/${name}.log") 2>&1
child_pid=""
cleanup() {
  trap - INT TERM
  if [[ -n "$child_pid" ]] && kill -0 "$child_pid" 2>/dev/null; then
    kill -INT "$child_pid" 2>/dev/null || true
    sleep 1
    kill -TERM "$child_pid" 2>/dev/null || true
  fi
  rm -f "$RUN_DIR/${name}.wrapper.pid" "$RUN_DIR/${name}.child.pid"
  exit 0
}
trap cleanup INT TERM
printf '\n===== %s | %s =====\n' "$name" "$(date '+%F %T')"
"$TOOLS_DIR/sitl_component.sh" "$name" </dev/tty &
child_pid=$!
echo "$child_pid" > "$RUN_DIR/${name}.child.pid"
wait "$child_pid"
rc=$?
rm -f "$RUN_DIR/${name}.child.pid" "$RUN_DIR/${name}.wrapper.pid"
printf '\n[%s] process exited with status %s. Log: %s/logs/%s.log\n' "$name" "$rc" "$RUN_DIR" "$name"
if [[ -t 0 ]]; then read -r -p "Press Enter to close this window..." _ || true; fi
exit "$rc"
