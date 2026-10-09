#!/usr/bin/env bash
# Isolated, headless simulation ONLY. Never arms or starts a mission.
set -eo pipefail
FLIGHT_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AP="$HOME/github/ardupilot"
GZ="$HOME/github/ardupilot_gazebo"
RUNTIME="$FLIGHT_WS/.sitl_runtime/flight_instance"
ADAPTER="${SITL_FLIGHT_COVARIANCE_ADAPTER:-false}"
[[ "$ADAPTER" == true || "$ADAPTER" == false ]] || exit 2
mkdir -p "$RUNTIME"
source /opt/ros/noetic/setup.bash
source "$FLIGHT_WS/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11321
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
unset ROS_IP ROS_HOSTNAME
# Exclusive ports: fail rather than attaching to somebody else's simulator.
python3 - <<'PY'
import socket
for kind,ports in ((socket.SOCK_STREAM,(11321,11355,5760)),(socket.SOCK_DGRAM,(9002,9003,14572))):
 for port in ports:
  s=socket.socket(socket.AF_INET,kind);s.bind(('0.0.0.0',port));s.close()
PY
cp "$AP/Tools/autotest/eeprom.bin" "$RUNTIME/eeprom.bin"
# Legacy Gazebo binary FDM drops intermediate packets; 400Hz input yielded ~304Hz FC loop.
sed 's#<max_step_size>0.0025</max_step_size>#<max_step_size>0.001</max_step_size>#' \
 "$GZ/worlds/iris_ardupilot.world" > "$RUNTIME/flight.world"
export GAZEBO_MODEL_PATH="$GZ/models:$GZ/models_gazebo:$AP/Tools/autotest/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$GZ/worlds:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="$GZ/build:${GAZEBO_PLUGIN_PATH:-}"
cd "$RUNTIME"
children=()
cleanup() { kill -INT "${children[@]}" 2>/dev/null || true; wait || true; }
trap cleanup EXIT INT TERM
roscore -p 11321 > roscore.log 2>&1 & children+=("$!"); echo "$!" > roscore.pid
for i in {1..30}; do rosparam list >/dev/null 2>&1 && break; sleep 1; done
"$AP/build/sitl/bin/arducopter" -S --model gazebo --speedup 1 \
 --defaults "$AP/Tools/autotest/default_params/copter.parm,$AP/Tools/autotest/default_params/gazebo-iris.parm" \
 --home=-35.363261,149.165230,584,0 -I0 > arducopter.log 2>&1 & children+=("$!"); echo "$!" > arducopter.pid
roslaunch gazebo_ros empty_world.launch world_name:="$RUNTIME/flight.world" \
 paused:=false gui:=false use_sim_time:=false verbose:=true > gazebo.log 2>&1 & children+=("$!"); echo "$!" > gazebo.pid
mavproxy.py --master=tcp:127.0.0.1:5760 --out=127.0.0.1:14572 --streamrate=25 \
 --source-system=245 --source-component=190 --daemon > mavproxy.log 2>&1 & children+=("$!"); echo "$!" > mavproxy.pid
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14572@ gcs_url:= > mavros.log 2>&1 & children+=("$!"); echo "$!" > mavros.pid
roslaunch indoor_nav indoor_nav_sitl.launch external_nav_enabled:=true sitl_covariance_adapter:="$ADAPTER" mission_file:="$FLIGHT_WS/src/indoor_nav/config/mission_sitl_flight.yaml" > indoor.log 2>&1 & children+=("$!"); echo "$!" > indoor.pid
printf '%s\n' "$ROS_MASTER_URI" > master_uri
# This job remains owned by its user systemd service, including descendants.
wait -n
