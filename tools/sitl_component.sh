#!/usr/bin/env bash
# 普通图形终端的组件启动器；由 window_component.sh 调用。
set -Eeuo pipefail
TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$TOOLS_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$TOOLS_DIR/sitl.env"
source /opt/ros/noetic/setup.bash
if [[ ! -f "$WS_ROOT/devel/setup.bash" ]]; then
  echo "[ERROR] 缺少 $WS_ROOT/devel/setup.bash，请先 catkin_make。" >&2
  exit 2
fi
source "$WS_ROOT/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:11311"
unset ROS_IP ROS_HOSTNAME || true

export GAZEBO_MODEL_PATH="$GAZEBO_ROOT/models:$GAZEBO_ROOT/models_gazebo:$AP_ROOT/Tools/autotest/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$GAZEBO_ROOT/worlds:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="$GAZEBO_ROOT/build:${GAZEBO_PLUGIN_PATH:-}"
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  # setup.sh 部分版本会重设 Gazebo 环境变量，所以先加载再追加路径
  source /usr/share/gazebo/setup.sh
  export GAZEBO_MODEL_PATH="$GAZEBO_ROOT/models:$GAZEBO_ROOT/models_gazebo:$AP_ROOT/Tools/autotest/models:${GAZEBO_MODEL_PATH:-}"
  export GAZEBO_RESOURCE_PATH="$GAZEBO_ROOT/worlds:${GAZEBO_RESOURCE_PATH:-}"
  export GAZEBO_PLUGIN_PATH="$GAZEBO_ROOT/build:${GAZEBO_PLUGIN_PATH:-}"
fi

component="${1:-}"
echo "[indoor_nav] component=$component  ws=$WS_ROOT"
case "$component" in
  roscore)
    exec roscore
    ;;
  gazebo)
    echo "[Gazebo11] world=$WORLD_FILE"
    # 关键：使用 gazebo_ros 的入口提供 /gazebo/model_states，避免裸 gazebo 无 ROS API。
    exec roslaunch gazebo_ros empty_world.launch \
      "world_name:=$WORLD_FILE" \
      paused:=false gui:="$GAZEBO_GUI" use_sim_time:=false verbose:=true
    ;;
  sitl)
    cd "$AP_ROOT/Tools/autotest"
    echo "[ArduPilot] frame=$SITL_FRAME model=$SITL_MODEL out=$MAVROS_UDP_PORT"
    # --model gazebo: legacy Gazebo Classic binary backend, not JSON.
    sitl_opts=()
    if [[ "$SITL_CONSOLE" == "true" ]]; then sitl_opts+=(--console); fi
    if [[ "$SITL_MAP" == "true" ]]; then sitl_opts+=(--map); fi
    exec python3 sim_vehicle.py \
      -v ArduCopter -f "$SITL_FRAME" --model "$SITL_MODEL" \
      "${sitl_opts[@]}" --out="127.0.0.1:$MAVROS_UDP_PORT"
    ;;
  mavros)
    echo "[MAVROS] listening udp://:$MAVROS_UDP_PORT@"
    exec roslaunch mavros apm.launch \
      "fcu_url:=udp://:$MAVROS_UDP_PORT@" "gcs_url:="
    ;;
  indoor)
    echo "[indoor_nav] ExternalNav forwarding=$EXTERNAL_NAV_ENABLED (default false)"
    exec roslaunch indoor_nav indoor_nav_sitl.launch \
      "external_nav_enabled:=$EXTERNAL_NAV_ENABLED" \
      "gazebo_model_name:=$GAZEBO_MODEL_NAME" \
      "uwb_noise_std_xy_m:=$UWB_NOISE_STD_XY_M" \
      "uwb_delay_ms:=$UWB_DELAY_MS"
    ;;
  qgc)
    if [[ ! -x "$QGC_APP" ]]; then
      echo "[WARN] QGC 不存在或不可执行：$QGC_APP" >&2
      exit 3
    fi
    exec "$QGC_APP"
    ;;
  *)
    echo "Unknown component: '$component'" >&2
    exit 2
    ;;
esac
