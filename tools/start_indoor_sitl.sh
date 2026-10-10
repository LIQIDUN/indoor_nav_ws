#!/usr/bin/env bash
# 参考 start_quadtilt.sh：每个组件直接运行在独立的 Terminator 终端里。
# 启动顺序：roscore -> SITL -> Gazebo11 -> MAVROS -> indoor_nav -> (QGC)。
set -Eeuo pipefail

TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$TOOLS_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$TOOLS_DIR/sitl.env"
RUN_DIR="$WS_ROOT/.sitl_runtime"
mkdir -p "$RUN_DIR"
GUI_DIR="$RUN_DIR/gui_instance"

fail() { echo "[ERROR] $*" >&2; exit 1; }
info() { echo "[indoor SITL] $*"; }
q() { printf '%q' "$1"; }

[[ -f "$WS_ROOT/devel/setup.bash" ]] || fail "缺少 $WS_ROOT/devel/setup.bash：请先 catkin_make"
[[ -f "$AP_ROOT/Tools/autotest/sim_vehicle.py" ]] || fail "未找到 ArduPilot：$AP_ROOT"
[[ -f "$WORLD_FILE" ]] || fail "未找到 Gazebo world：$WORLD_FILE"
[[ -f "$WS_ROOT/src/indoor_nav/launch/indoor_nav_sitl.launch" ]] || fail '未找到 indoor_nav_sitl.launch'
[[ "${SITL_MODEL:-gazebo}" == gazebo ]] || fail 'Gazebo11 Classic 必须使用 SITL_MODEL=gazebo，而不是 JSON'
# MAVProxy owns telemetry stream rates; this section does not change EKF parameters.
[[ "${SITL_MAVLINK_STREAMRATE:-25}" =~ ^[0-9]+$ ]] || fail 'SITL_MAVLINK_STREAMRATE must be 1..50 Hz'
(( SITL_MAVLINK_STREAMRATE >= 1 && SITL_MAVLINK_STREAMRATE <= 50 )) || fail 'SITL_MAVLINK_STREAMRATE must be 1..50 Hz'
if (( SITL_MAVLINK_STREAMRATE < 20 )); then echo "[WARN] Telemetry below 20Hz: diagnostic rollback only; not flight acceptance." >&2; fi

[[ "$SITL_COVARIANCE_ADAPTER" == true || "$SITL_COVARIANCE_ADAPTER" == false ]] || fail "Invalid SITL_COVARIANCE_ADAPTER"
[[ -f "$SITL_MISSION_FILE" ]] || fail "Missing mission: $SITL_MISSION_FILE"

[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || fail '未检测到图形桌面；本脚本使用 Terminator 独立窗口'
command -v terminator >/dev/null || fail '未安装 terminator：sudo apt install terminator'
set +u  # ROS setup scripts read optional unset environment variables
source /opt/ros/noetic/setup.bash
source "$WS_ROOT/devel/setup.bash"
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
unset ROS_IP ROS_HOSTNAME

# 和参考脚本一样：启动前清理本项目旧实例；不执行 pkill -9 -f ros。
if [[ -f "$RUN_DIR/session.running" ]] || compgen -G "$RUN_DIR/*.terminal.pid" >/dev/null; then
  info '发现上次会话，先清理旧进程...'
  "$TOOLS_DIR/stop_indoor_sitl.sh"
fi
if timeout 2 bash -c 'echo > /dev/tcp/127.0.0.1/11311' >/dev/null 2>&1; then
  fail '11311 上已有 ROS Master（可能来自其他项目）；为避免误伤，先手动确认并关闭它'
fi

# Fail before creating terminals if another SITL/Gazebo instance owns shared ports.
python3 - "$MAVROS_UDP_PORT" <<'PYPORT'
import socket, sys
for kind, ports in ((socket.SOCK_STREAM, (11345, 5760)),
                    (socket.SOCK_DGRAM, (9002, 9003, int(sys.argv[1])))):
    for port in ports:
        with socket.socket(socket.AF_INET, kind) as sock:
            sock.bind(('0.0.0.0', port))
PYPORT
mkdir -p "$GUI_DIR"
# Never edit external world or EEPROM. Cold starts use a fresh copy of the same seed as run_06.
python3 - "$WORLD_FILE" "$GUI_DIR/flight.world" "$SITL_PHYSICS_STEP_S" <<'PYWORLD'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1])
steps = root.findall('.//physics/max_step_size')
if len(steps) != 1:
    raise SystemExit('Expected exactly one Gazebo physics/max_step_size')
steps[0].text = sys.argv[3]
root.write(sys.argv[2], encoding='utf-8', xml_declaration=True)
PYWORLD
cp "$AP_ROOT/Tools/autotest/eeprom.bin" "$GUI_DIR/eeprom.bin"
WORLD_FILE="$GUI_DIR/flight.world"
# Record early so normal stop also works after a partial startup failure.
printf '%s\n' "$AP_ROOT" > "$RUN_DIR/session.ap_root"
printf '%s\n' "$WORLD_FILE" > "$RUN_DIR/session.world_file"
printf '%s\n' "$MAVROS_UDP_PORT" > "$RUN_DIR/session.mavros_port"
date '+%F %T' > "$RUN_DIR/session.running"

# 只使用原生 shell + 原始命令；没有 tmux/window_component/sitl_component 包装层。
# 命令结束时保留交互式 bash，方便直接查看错误信息。
open_terminal() {
  local key="$1" geo="$2" title="INDOOR-UWB $3" command="$4"
  info "启动 $3：$title"
  terminator -u --geometry="$geo" -T "$title" -x bash -c \
    "echo \$BASHPID > $(q "$RUN_DIR/$key.shell.pid"); $command; ret=\$?; echo; echo '[indoor SITL] 组件已退出，状态码:' \$ret; exec bash" &
  echo "$!" > "$RUN_DIR/$key.terminal.pid"
}

wait_for() {
  local name="$1" attempts="$2" test_cmd="$3" i
  for ((i=1; i<=attempts; i++)); do
    if eval "$test_cmd" >/dev/null 2>&1; then info "$name 已就绪"; return 0; fi
    sleep 1
  done
  fail "$name 未就绪；请直接查看对应的终端报错，并运行 stop_indoor_sitl.sh 清理"
}

# 所有路径与参数都通过 shell 转义，以便目录中有空格时也能工作。
ROS_ENV="source /opt/ros/noetic/setup.bash && source $(q "$WS_ROOT/devel/setup.bash") && export ROS_MASTER_URI=http://127.0.0.1:11311 && export GAZEBO_MASTER_URI=http://127.0.0.1:11345 && unset ROS_IP ROS_HOSTNAME"
SITL_OPTIONS="--use-dir=$(q "$GUI_DIR") --custom-location=$(q "$SITL_HOME") --mavproxy-args=$(q "--streamrate=$SITL_MAVLINK_STREAMRATE")"
[[ "${SITL_CONSOLE:-false}" == true ]] && SITL_OPTIONS+=' --console'
[[ "${SITL_MAP:-false}" == true ]] && SITL_OPTIONS+=' --map'

# 1. ROS Core，标准原始输出可见
open_terminal roscore '600x320+0+0' 'ROS Master' \
  "$ROS_ENV && echo '[RUN] roscore' && roscore"
wait_for 'ROS Master' 30 "timeout 2 rostopic list | grep -qx /rosout"

# 2. SITL 在 Gazebo 之前启动；sim_vehicle.py 可能另开一个原生 ArduCopter 终端。
open_terminal sitl '850x430+0+340' 'ArduPilot SITL / MAVProxy' \
  "cd $(q "$AP_ROOT/Tools/autotest") && echo '[RUN] sim_vehicle.py -v ArduCopter -f $(q "$SITL_FRAME") --model gazebo --out 127.0.0.1:$MAVROS_UDP_PORT' && python3 sim_vehicle.py -v ArduCopter -f $(q "$SITL_FRAME") --model gazebo $SITL_OPTIONS --out=$(q "127.0.0.1:$MAVROS_UDP_PORT")"
wait_for 'ArduPilot UDP 9003' 120 "ss -H -lun | grep -Eq '(:|\\])9003[[:space:]]'"

# 3. Gazebo11，gazebo_ros 提供 /gazebo/model_states
GAZEBO_ENV=""
if [[ -f /usr/share/gazebo/setup.sh ]]; then GAZEBO_ENV='source /usr/share/gazebo/setup.sh && '; fi
open_terminal gazebo '800x420+850+0' 'Gazebo11' \
  "$ROS_ENV && ${GAZEBO_ENV}export GAZEBO_MODEL_PATH=$(q "$GAZEBO_ROOT/models:$GAZEBO_ROOT/models_gazebo:$AP_ROOT/Tools/autotest/models"):\${GAZEBO_MODEL_PATH:-} && export GAZEBO_RESOURCE_PATH=$(q "$GAZEBO_ROOT/worlds"):\${GAZEBO_RESOURCE_PATH:-} && export GAZEBO_PLUGIN_PATH=$(q "$GAZEBO_ROOT/build"):\${GAZEBO_PLUGIN_PATH:-} && cd $(q "$GAZEBO_ROOT/worlds") && echo '[RUN] roslaunch gazebo_ros empty_world.launch' && roslaunch gazebo_ros empty_world.launch world_name:=$(q "$WORLD_FILE") paused:=false gui:=$(q "$GAZEBO_GUI") use_sim_time:=false verbose:=true"
wait_for 'Gazebo /gazebo/model_states' 90 "timeout 2 rostopic list | grep -qx /gazebo/model_states"

# 4. MAVROS，使用原生 roslaunch 窗口
open_terminal mavros '740x430+850+440' 'MAVROS' \
  "$ROS_ENV && echo '[RUN] roslaunch mavros apm.launch' && roslaunch mavros apm.launch $(q "fcu_url:=udp://127.0.0.1:$MAVROS_UDP_PORT@") gcs_url:="
# 输出等待情况；不能把话题存在误当成 MAVROS 已经连接
mavros_ok=0
for ((i=1; i<=45; i++)); do
  if timeout 3 rostopic echo -n1 /mavros/state 2>/dev/null | grep -Eq '^connected: [Tt]rue$'; then
    mavros_ok=1; break
  fi
  sleep 1
done
if [[ "$mavros_ok" == 1 ]]; then
  info 'MAVROS connected=True'
else
  info '警告：MAVROS 尚未 connected=True；终端均保留，继续启动室内定位节点便于调试'
fi

# 5. indoor_nav：保持当前安全配置，不启动 ARM、不启动 Mission
open_terminal indoor '800x430+1600+0' 'indoor_nav' \
  "$ROS_ENV && echo '[RUN] roslaunch indoor_nav indoor_nav_sitl.launch' && roslaunch indoor_nav indoor_nav_sitl.launch $(q "external_nav_enabled:=$EXTERNAL_NAV_ENABLED") $(q "sitl_covariance_adapter:=$SITL_COVARIANCE_ADAPTER") $(q "mission_file:=$SITL_MISSION_FILE") $(q "gazebo_model_name:=$GAZEBO_MODEL_NAME") $(q "uwb_noise_std_xy_m:=$UWB_NOISE_STD_XY_M") $(q "uwb_delay_ms:=$UWB_DELAY_MS")"

# 6. QGroundControl：默认启动；允许 sitl.env 中 START_QGC=0 关闭。
# 优先使用显式 QGC_APP，随后寻找 PATH 或常见 AppImage 安装位置。
if [[ "${START_QGC:-1}" == 1 ]]; then
  QGC_BIN=""
  if [[ -n "${QGC_APP:-}" && -x "${QGC_APP}" ]]; then
    QGC_BIN="$QGC_APP"
  elif command -v QGroundControl >/dev/null 2>&1; then
    QGC_BIN="$(command -v QGroundControl)"
  elif command -v qgroundcontrol >/dev/null 2>&1; then
    QGC_BIN="$(command -v qgroundcontrol)"
  else
    for candidate in "$HOME/Applications/QGroundControl.AppImage" "$HOME/Downloads/QGroundControl.AppImage" \
                     "$HOME/QGroundControl.AppImage" "/opt/QGroundControl/QGroundControl.AppImage"; do
      if [[ -x "$candidate" ]]; then QGC_BIN="$candidate"; break; fi
    done
  fi
  if [[ -n "$QGC_BIN" ]]; then
    open_terminal qgc '650x360+1600+470' 'QGroundControl' \
      "echo '[RUN] QGroundControl: $(q "$QGC_BIN")' && $(q "$QGC_BIN")"
  else
    info '未找到可执行 QGroundControl。请在 tools/sitl.env 设置 QGC_APP=/完整路径/QGroundControl.AppImage'
  fi
fi

info '准备SITL心跳、只读EKF检查及origin回读；不会校准、ACK、解锁或起飞。'
if ! timeout 120 python3 "$TOOLS_DIR/sitl_gui_prepare.py" --ap-root "$AP_ROOT" --home="$SITL_HOME" --port "$MAVROS_UDP_PORT"; then
  fail 'SITL准备失败：不要飞行。查看 .sitl_runtime/gui_instance/prepare.json 和组件终端；可用原stop脚本关闭。'
fi
info '全部组件已发起启动。各自终端可直接查看原始 ROS/SITL/Gazebo 日志。'
info "ExternalNav=$EXTERNAL_NAV_ENABLED；不会自动 ARM 或起飞。"
info '关闭：./tools/stop_indoor_sitl.sh'
