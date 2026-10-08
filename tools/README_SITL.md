# indoor_nav Gazebo Classic / SITL — ordinary terminal windows

This replaces the previous **tmux** runner. It opens separate Terminator/gnome-terminal windows, uses the **SITL → Gazebo → MAVROS → indoor_nav** startup order, and saves component logs to `.sitl_runtime/logs/`.

## Important: gazebo vs JSON

The bundled `iris_ardupilot.world` is the **Gazebo Classic** / SwiftGust (or khancyr) ArduPilotPlugin using binary servo/sensor packets on UDP **9002/9003**. SITL must be run with `--model gazebo` rather than `--model JSON`. The latter is for modern Gazebo Sim's JSON plugin. Using JSON with Classic leads to `No JSON sensor message received`, `Broken ArduPilot connection`, and MAVROS `connected: False`.

## Install into existing workspace

```bash
cd ~/github/indoor_nav_ws
unzip -o ~/Downloads/indoor_nav_sitl_terminals.zip -d .
chmod +x tools/*.sh
```

No source code or ROS launch files are overwritten by this ZIP. Existing `src/indoor_nav` stays unchanged.

**First stop the previous tmux session manually before changing launchers** (old tmux-based stop script will be replaced):

```bash
tmux has-session -t indoor_uwb_sitl && tmux kill-session -t indoor_uwb_sitl
```

Check no stale `gzserver`, `arducopter`, `mavproxy.py`, or `roscore` are still running. If present, stop only those related to the old session; don't blindly `pkill` all ROS tasks.

## Start

```bash
cd ~/github/indoor_nav_ws
./tools/start_indoor_sitl.sh
```

Actual order: ROS Master → SITL/ArduCopter/MAVProxy → Gazebo11 (with `/gazebo/model_states`) → MAVROS (wait for **connected=True**) → indoor_nav. QGC optionally starts last.

`sim_vehicle.py` may open one extra ArduCopter console window itself. This is expected.

The window title indicates the component. If a component exits, its window stays open until Enter is pressed. The stdout logs are also saved to `.sitl_runtime/logs/`.

If MAVROS does not connect, the launcher intentionally does not start `indoor_nav` and leaves consoles visible for diagnosis.

## Stop

```bash
./tools/stop_indoor_sitl.sh
```

Stop sends signals only to tracked component wrapper PIDs and the exact ArduCopter PID captured for this session. It does **not** broadly kill other workspaces' processes. In abnormal cases, check the process list and shut down detached Gazebo/MAVProxy windows manually.

## Tests

```bash
rostopic echo -n1 /mavros/state
rostopic echo -n1 /gazebo/model_states
rostopic echo -n1 /nlink_linktrack_nodeframe2
rostopic echo -n1 /indoor_nav/localization_status
```

Do **not** arm the aircraft or run a mission yet. First verify frame alignment, EKF source selection and the ExternalNav path.

## Parameters

Edit `tools/sitl.env` for repo locations, world/model name, `TERMINAL_APP`, QGC, and ExternalNav forwarding. Keep `SITL_MODEL=gazebo` for Gazebo11/Classic. The default `EXTERNAL_NAV_ENABLED=false` is intentional.
