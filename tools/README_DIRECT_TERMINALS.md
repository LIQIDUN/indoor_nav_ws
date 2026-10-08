# indoor_nav SITL：独立终端启动/彻底关闭版

根据你的 `start_quadtilt.sh` 和 `stop_quadtilt.sh` 改造。所有输出直接显示在每个 Terminator 终端中，不使用 tmux，也不经过 `window_component.sh` / `sitl_component.sh`。

## 重要改动

- **原生窗口**：ROS Master 直接执行 `roscore`；SITL 直接执行 `python3 sim_vehicle.py`；Gazebo/MAVROS/indoor_nav 直接执行 `roslaunch`。
- **顺序**：ROS Master → ArduPilot SITL（等待 UDP 9003）→ Gazebo11（等待 `/gazebo/model_states`）→ MAVROS → indoor_nav；可选 QGC。
- **协议**：Gazebo11 Classic 的 ArduPilot 插件使用 `--model gazebo`，**不能改成 `--model JSON`**。
- **退出显示**：如果子命令退出，终端仍会进入 bash，保留错误信息。
- **关闭**：先向真实业务组件发送 INT/TERM，再针对启动时记录的进程树补救；清理 Terminator 窗口。
- **不会触碰**：当前 `sitl.env`、`safety_sitl.yaml`、ROS launch、ArduPilot 源码和编译产物。
- **安全默认**：`EXTERNAL_NAV_ENABLED=false`、不自动 ARM/起飞、不启动 mission。

## 安装

```bash
cd ~/github/indoor_nav_ws

# 备份旧版脚本
cp tools/start_indoor_sitl.sh tools/start_indoor_sitl.sh.bak
cp tools/stop_indoor_sitl.sh tools/stop_indoor_sitl.sh.bak

# 假设 ZIP 在 ~/Downloads
unzip -o ~/Downloads/indoor_nav_direct_terminals.zip -d .
chmod +x tools/start_indoor_sitl.sh tools/stop_indoor_sitl.sh
```

**压缩包仅覆盖 `start_indoor_sitl.sh`、`stop_indoor_sitl.sh`，新增这份 README；不覆盖 `sitl.env`。** 之前的 `window_component.sh`、`sitl_component.sh` 可以保留，但新启动器不会再调用它们。

## 运行

```bash
cd ~/github/indoor_nav_ws
./tools/stop_indoor_sitl.sh   # 建议首次先清一次旧会话
./tools/start_indoor_sitl.sh
```

你可以直接在 **ROS Master** 终端看到 `roscore` 的输出，在 **SITL** 终端看到 MAVProxy 原生交互控制台，在 **Gazebo11** 终端看到插件和 world 加载日志。`sim_vehicle.py` 还可能自行打开一个 ArduCopter 终端，这是 ArduPilot 自带行为，不是我们的包装层。

## 关闭

```bash
./tools/stop_indoor_sitl.sh
```

脚本会报告残余进程和 9002/9003/14552 端口。如果仍有独立进程，**且你确认当前电脑没有其他 ROS/Gazebo/SITL 仿真需要保留**，再执行：

```bash
./tools/stop_indoor_sitl.sh --force
```

`--force` 按进程名清理所有 `gzserver`、`gzclient`、`arducopter`、`mavros_node`、`roscore`、`rosmaster`，可能误杀其他项目；正常情况下不需要。与参考脚本不同，默认不会全局执行 `pkill -9 -f ros`。

## 快速验证

```bash
rostopic echo -n1 /mavros/state
rostopic echo -n1 /gazebo/model_states
rostopic echo -n1 /indoor_nav/localization_status
```

如果之前已经校准：**只有重新启动了 `world_estimator` 节点才需要重新调用 `/indoor_nav/calibrate`**。安全确认 `/indoor_nav/safety/acknowledge` 也是节点重启后重新执行。

## 约定与局限

必须有 Terminator、Gazebo11、ROS Noetic 以及原工作空间的安装。这个版本专门用于你的 Ubuntu 图形桌面，不支持无显示器的 SSH/headless。关闭逻辑使用会话记录的进程树和本项目路径；极端情况下 detached 的 GUI 子进程可能需要 `--force`。测试前确认 `sitl.env` 中 `AP_ROOT`、`WORLD_FILE`、`SITL_MODEL=gazebo` 和 `MAVROS_UDP_PORT` 正确。
