# 用户暂停指令后的本轮实验停止记录

- 日期：2026-10-26，Asia/Shanghai。
- 范围：只停止本轮隔离SITL，不启动新实验，不操作真机，不停止已有旧MAVROS。
- 来源：本次停止工具与主机级只读核查的终端输出整理，不是先前 `final_ports.txt` 的复用。

## 停止命令与结果

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11421 ROS_IP=127.0.0.1
python3 tools/v0_sitl_stop.py --sitl
```

工具要求已连接且DISARM，核对runtime/SITL executable/launcher父子身份后发送停止信号，退出码0：

```text
Requested PID-scoped disarmed SITL shutdown: 3535222 /home/liqidun/github/indoor_nav_ws/.sitl_runtime/v0/20261026T183647-3535222
```

## 停止后主机级核查

时间：`2026-10-26T19:09:31+08:00`。核查在主机权限下执行，未用沙箱内空进程列表代替。

- `ps -eo pid,ppid,etime,stat,args` 中未匹配本轮 `v0_sitl_instance.sh`、`tools/v0_sitl_test.py`、指定Gazebo模型ArduCopter、`relative_mission_manager`、`v0_nav_health.py`、`v0_mock_acquisition`、`rosbag record`。
- `ss -luntp` 未匹配 `:11421|:11455|:5760|:9002|:9003|:14672`。
- 精确检查本轮PID `3535222,3535364,3535448,3535491,3535296`，只有表头，无进程行；该精确ps无匹配返回1是预期现象。
- 以下旧进程仍在，未终止：

```text
PID2391064  Wed Oct 14 20:13:15 2026  /opt/ros/noetic/lib/mavros/mavros_node
PID3263919  Thu Oct 15 14:52:11 2026  /opt/ros/noetic/lib/mavros/mavros_node
```

结论：本轮仿真/实验已停止；不宣称主机全局无MAVROS或所有旧业务已停。没有启动补测。
