#!/usr/bin/env bash
# 参考 stop_quadtilt.sh：真实关闭 SITL / Gazebo / ROS / 终端，不仅仅关闭启动器。
# 默认只处理本项目启动的实例。 --force 才允许清理本机全部同名仿真组件。
set -u
TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$TOOLS_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$TOOLS_DIR/sitl.env"
RUN_DIR="$WS_ROOT/.sitl_runtime"
FORCE=0
[[ "${1:-}" == --force ]] && FORCE=1

info() { echo "[indoor SITL] $*"; }
cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null || true; }
alive() { [[ -d "/proc/$1" ]]; }

# 启动时记录的配置优先，防止之后修改 sitl.env 导致旧进程无法匹配。
[[ -f "$RUN_DIR/session.ap_root" ]] && AP_ROOT="$(cat "$RUN_DIR/session.ap_root")"
[[ -f "$RUN_DIR/session.world_file" ]] && WORLD_FILE="$(cat "$RUN_DIR/session.world_file")"
[[ -f "$RUN_DIR/session.mavros_port" ]] && MAVROS_UDP_PORT="$(cat "$RUN_DIR/session.mavros_port")"

# 保存已启动窗口的进程树快照：roslaunch 派生的 gzserver/gzclient、ROS 节点等。
# 后面即使 roslaunch 自己先退出，这些子进程仍能被收尾。
declare -a TRACKED_PIDS=()
declare -A SEEN=()
add_pid() {
  local pid="$1"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  [[ "$pid" != "$$" && "$pid" != "$PPID" ]] || return 0
  alive "$pid" || return 0
  if [[ -z "${SEEN[$pid]+x}" ]]; then
    SEEN[$pid]=1
    TRACKED_PIDS+=("$pid")
  fi
}
collect_descendants() {
  local root="$1" child
  [[ "$root" =~ ^[0-9]+$ ]] || return 0
  alive "$root" || return 0
  add_pid "$root"
  while IFS= read -r child; do
    [[ "$child" =~ ^[0-9]+$ ]] || continue
    [[ -z "${SEEN[$child]+x}" ]] || continue
    collect_descendants "$child"
  done < <(pgrep -P "$root" 2>/dev/null || true)
}

# 没有本项目启动记录时，默认不碰可能属于别的工作空间的 MAVROS/roscore。
if [[ "$FORCE" == 0 && ! -f "$RUN_DIR/session.running" ]] && \
   ! compgen -G "$RUN_DIR/*.terminal.pid" >/dev/null && \
   ! compgen -G "$RUN_DIR/*.wrapper.pid" >/dev/null && \
   ! compgen -G "$RUN_DIR/*.shell.pid" >/dev/null; then
  info '没有发现本项目的会话记录。若确认有旧版遗留进程，可运行 --force（会影响其他同名仿真）。'
  exit 0
fi

info '准备关闭 indoor_nav、MAVROS、Gazebo、ArduPilot SITL、ROS Master...'
# 兼容旧版 tmux 启动器的残留会话。
if command -v tmux >/dev/null 2>&1 && tmux has-session -t '=indoor_uwb_sitl' 2>/dev/null; then
  info '清理旧版 indoor_uwb_sitl tmux 会话'
  tmux kill-session -t '=indoor_uwb_sitl' 2>/dev/null || true
fi

# 新版独立终端的 shell / terminal PID；旧版 wrapper PID 也接受。
for component in indoor mavros gazebo sitl roscore qgc; do
  for suffix in shell terminal wrapper child; do
    file="$RUN_DIR/$component.$suffix.pid"
    [[ -f "$file" ]] || continue
    pid="$(cat "$file" 2>/dev/null || true)"
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    # 终端启动器可能在其他进程中复用 PID：只收集看起来仍是组件的进程。
    c="$(cmdline "$pid")"
    if [[ "$suffix" == terminal ]]; then
      [[ "$c" == *terminator* || "$c" == *gnome-terminal* ]] || continue
    elif [[ "$suffix" == wrapper ]]; then
      [[ "$c" == *window_component.sh* ]] || continue
    elif [[ "$suffix" == shell ]]; then
      [[ "$c" == *bash* || "$c" == *roscore* || "$c" == *roslaunch* ]] || continue
    fi
    collect_descendants "$pid"
  done
done

# 给业务主进程发 SIGINT，让 roslaunch 正常停止并清理它的子节点。
# ps comm 过滤能防止误中 bash -c 中写出的字符串。
signal_named_processes() {
  local sig="$1" pid cmd comm
  while IFS= read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    [[ "$pid" != "$$" && "$pid" != "$PPID" ]] || continue
    alive "$pid" || continue
    cmd="$(cmdline "$pid")"
    comm="$(ps -p "$pid" -o comm= 2>/dev/null | xargs || true)"
    case "$comm" in
      python3|python|roslaunch|mavproxy.py)
        if [[ "$cmd" == *'roslaunch indoor_nav indoor_nav_sitl.launch'* ||
              "$cmd" == *'roslaunch mavros apm.launch'* ||
              ( "$cmd" == *'roslaunch gazebo_ros empty_world.launch'* && "$cmd" == *"$WORLD_FILE"* ) ||
              ( "$cmd" == *'sim_vehicle.py -v ArduCopter'* && "$cmd" == *'--model gazebo'* ) ||
              ( "$cmd" == *'mavproxy.py'* && "$cmd" == *"127.0.0.1:$MAVROS_UDP_PORT"* && "$cmd" == *'5760'* ) ||
              ( "$cmd" == *"$WS_ROOT/devel/lib/indoor_nav/"* ) ]]; then
          kill "-$sig" "$pid" 2>/dev/null || true
        fi
        ;;
      arducopter)
        if [[ "$cmd" == "$AP_ROOT/build/sitl/bin/arducopter "* ]]; then
          kill "-$sig" "$pid" 2>/dev/null || true
        fi
        ;;
      gzserver)
        if [[ "$cmd" == *"$WORLD_FILE"* ]]; then
          kill "-$sig" "$pid" 2>/dev/null || true
        fi
        ;;
    esac
  done < <(pgrep -u "$(id -u)" -f 'roslaunch|sim_vehicle.py|mavproxy.py|arducopter|gzserver|/indoor_nav/' 2>/dev/null || true)
}

signal_named_processes INT
# 对记录的 roslaunch/SITL 组件子进程发 SIGINT（保留终端外壳，最后再关窗口）。
for pid in "${TRACKED_PIDS[@]}"; do
  comm="$(ps -p "$pid" -o comm= 2>/dev/null | xargs || true)"
  case "$comm" in
    roslaunch|roscore|rosmaster|python3|python|arducopter|gzserver|gzclient|mavros_node|QGroundControl|QGroundControl.AppImage)
      kill -INT "$pid" 2>/dev/null || true ;;
  esac
done
sleep 3

# 兜底 TERM：处理未随 roslaunch / sim_vehicle 自动退出的进程。
signal_named_processes TERM
for pid in "${TRACKED_PIDS[@]}"; do
  if alive "$pid"; then
    comm="$(ps -p "$pid" -o comm= 2>/dev/null | xargs || true)"
    case "$comm" in
      roslaunch|roscore|rosmaster|python3|python|arducopter|gzserver|gzclient|mavros_node|QGroundControl|QGroundControl.AppImage)
        kill -TERM "$pid" 2>/dev/null || true ;;
    esac
  fi
done
sleep 2

# 结束独立终端：标题专属于 INDOOR-UWB，不会按名称杀掉全部 Terminator。
for component in indoor mavros gazebo sitl roscore qgc; do
  file="$RUN_DIR/$component.terminal.pid"
  if [[ -f "$file" ]]; then
    pid="$(cat "$file" 2>/dev/null || true)"
    if [[ "$pid" =~ ^[0-9]+$ ]] && alive "$pid"; then
      c="$(cmdline "$pid")"
      if [[ "$c" == *terminator*"INDOOR-UWB"* || "$c" == *terminator*"--geometry="* ]]; then
        kill -TERM "$pid" 2>/dev/null || true
      fi
    fi
  fi
  file="$RUN_DIR/$component.shell.pid"
  if [[ -f "$file" ]]; then
    pid="$(cat "$file" 2>/dev/null || true)"
    if [[ "$pid" =~ ^[0-9]+$ ]] && alive "$pid"; then
      c="$(cmdline "$pid")"
      [[ "$c" == *bash* ]] && kill -TERM "$pid" 2>/dev/null || true
    fi
  fi
done

# 最后兜底：只向本项目记录的仍存活子进程发送 KILL。
sleep 1
for pid in "${TRACKED_PIDS[@]}"; do
  if alive "$pid"; then
    comm="$(ps -p "$pid" -o comm= 2>/dev/null | xargs || true)"
    case "$comm" in
      roslaunch|roscore|rosmaster|python3|python|arducopter|gzserver|gzclient|mavros_node|QGroundControl|QGroundControl.AppImage)
        kill -KILL "$pid" 2>/dev/null || true ;;
    esac
  fi
done

# 可选强力模式，与用户的 stop_quadtilt.sh 类似。会影响其他同名仿真进程。
if [[ "$FORCE" == 1 ]]; then
  info '执行 --force：清理本机同名仿真进程（可能影响其他 ROS/Gazebo 实例）'
  for name in gzserver gzclient arducopter mavros_node roscore rosmaster; do
    pkill -TERM -x "$name" 2>/dev/null || true
  done
  sleep 1
  for name in gzserver gzclient arducopter mavros_node roscore rosmaster; do
    pkill -KILL -x "$name" 2>/dev/null || true
  done
fi

# 不删除 runtime 目录内其他日志；只清理本次会话标记。
rm -f "$RUN_DIR"/*.shell.pid "$RUN_DIR"/*.terminal.pid "$RUN_DIR"/*.wrapper.pid "$RUN_DIR"/*.child.pid \
  "$RUN_DIR"/arducopter.pid "$RUN_DIR"/session.running "$RUN_DIR"/session.ap_root \
  "$RUN_DIR"/session.world_file "$RUN_DIR"/session.mavros_port
info '关闭流程结束。检查残留进程与 UDP 端口：'
ss -lunp 2>/dev/null | grep -E ':(9002|9003|14552)[[:space:]]' || true
pgrep -af 'sim_vehicle.py|arducopter|mavproxy.py|gzserver|gzclient|roscore|mavros_node' || true
info '若仅有残留进程且确定不运行其他 ROS/Gazebo 项目，可执行：./tools/stop_indoor_sitl.sh --force'
