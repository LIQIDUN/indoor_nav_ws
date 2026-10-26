#!/usr/bin/env bash
set -eo pipefail
[[ "${1:-}" == --sitl ]] || { echo 'SITL ONLY: use --sitl'; exit 2; }
PROFILE="${2:-gps}"
if [[ "$PROFILE" == --f0-externalnav ]]; then PROFILE=externalnav; fi
[[ "$PROFILE" == gps || "$PROFILE" == externalnav ]] || exit 2
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AP="$HOME/github/ardupilot"
GZ="$HOME/github/ardupilot_gazebo"
RUNTIME="$WS/.sitl_runtime/v0/$(date +%Y%m%dT%H%M%S)-$$"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11421
export ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11455
unset DISPLAY ROS_HOSTNAME
python3 - <<'PY'
import socket
for kind, ports in ((socket.SOCK_STREAM, (11421,11455,5760)), (socket.SOCK_DGRAM,(9002,9003,14672))):
    for port in ports:
        probe = socket.socket(socket.AF_INET, kind)
        probe.bind(('0.0.0.0', port))
        probe.close()
PY
mkdir -p "$RUNTIME"
sed 's#<max_step_size>0.0025</max_step_size>#<max_step_size>0.001</max_step_size>#' \
  "$GZ/worlds/iris_ardupilot.world" > "$RUNTIME/flight.world"
DEFAULTS="$AP/Tools/autotest/default_params/copter.parm,$AP/Tools/autotest/default_params/gazebo-iris.parm"
if [[ "$PROFILE" == externalnav ]]; then
  sha256sum "$AP/Tools/autotest/eeprom.bin" > "$RUNTIME/seed.sha256"
  cp "$AP/Tools/autotest/eeprom.bin" "$RUNTIME/eeprom.bin"
else
  DEFAULTS="$DEFAULTS,$WS/src/indoor_nav/config/v0_sitl_gps.parm"
fi
sha256sum "$AP/build/sitl/bin/arducopter" "$WS/src/indoor_nav/config/v0_sitl_gps.parm" "$RUNTIME/flight.world" > "$RUNTIME/inputs.sha256"
export GAZEBO_MODEL_PATH="$GZ/models:$GZ/models_gazebo:$AP/Tools/autotest/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$GZ/worlds:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="$GZ/build:${GAZEBO_PLUGIN_PATH:-}"
export ROS_LOG_DIR="$RUNTIME/ros_logs"
cd "$RUNTIME"
children=()
sitl_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$sitl_pid" ]]; then
    kill -TERM "$sitl_pid" 2>/dev/null || true
    sleep 2
    kill -KILL "$sitl_pid" 2>/dev/null || true
  fi
  kill -INT "${children[@]}" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT INT TERM
roscore -p 11421 > roscore.log 2>&1 & children+=("$!")
for attempt in {1..30}; do rosparam list >/dev/null 2>&1 && break; sleep 1; done
rosparam set /v0_sitl/isolated true
rosparam set /v0_sitl/runtime "$RUNTIME"
rosparam set /v0_sitl/profile "$PROFILE"
"$AP/build/sitl/bin/arducopter" -S --model gazebo --speedup 1 \
 --defaults "$DEFAULTS" \
 --home=-35.363261,149.165230,584,0 -I0 > arducopter.log 2>&1 & children+=("$!")
echo "$!" > arducopter.pid
sitl_pid="$!"
roslaunch gazebo_ros empty_world.launch world_name:="$RUNTIME/flight.world" \
 paused:=false gui:=false use_sim_time:=false verbose:=true > gazebo.log 2>&1 & children+=("$!")
mavproxy.py --master=tcp:127.0.0.1:5760 --out=127.0.0.1:14672 --streamrate=25 \
 --source-system=245 --source-component=190 --daemon > mavproxy.log 2>&1 & children+=("$!")
echo "$!" > mavproxy.pid
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14672@ gcs_url:= > mavros.log 2>&1 & children+=("$!")
echo "V0 SITL ready to inspect: $RUNTIME (no ARM, no navigation started)"
if [[ "$PROFILE" == externalnav ]]; then
  roslaunch indoor_nav indoor_nav_sitl.launch external_nav_enabled:=true sitl_covariance_adapter:=true \
   mission_file:="$WS/src/indoor_nav/config/mission_sitl_flight.yaml" > indoor.log 2>&1 & children+=("$!")
fi
wait -n
