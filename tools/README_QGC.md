# QGroundControl 集成

新版 `start_indoor_sitl.sh` 默认启动 QGroundControl，并在 `stop_indoor_sitl.sh` 中与其他组件一同关闭。

在 `tools/sitl.env` 中可选设置：

```bash
START_QGC=1
QGC_APP="$HOME/Applications/QGroundControl.AppImage"
```

若 QGC 已安装到 PATH，或 AppImage 位于 `$HOME/Applications/`、`$HOME/Downloads/`、`$HOME/`、`/opt/QGroundControl/`，则可以自动找到。关闭自动启动设置 `START_QGC=0`。

ArduPilot `sim_vehicle.py` 默认通过 MAVProxy 发送到 `127.0.0.1:14550` 供 QGC 使用；MAVROS 使用独立 UDP 14552。

注意：若 `sitl.env` 原有 `START_QGC=0`，需要显式改为 1。关闭脚本只应关闭本项目启动的 QGC；请检查本机是否同时运行其他 QGC 实例。
