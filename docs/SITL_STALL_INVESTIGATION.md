# SITL/MAVROS 停滞调查（2026-10-09）

## 证据

复用 `FREEZE_INVESTIGATION.md` 和 `evidence/freeze_live_20261009.json`。该会话是用户上次实验后没有关闭的遗留仿真；进程存在本身无需额外解释。本轮没有重启、attach 调试器、改端口或改 ArduPilot/Gazebo 源码。

时间线：14:02:58.567 POSSIBLE_UWB_FREEZE→ABORT，14:02:58.569 请求LAND，约14:03:00最后IMU/速度/DataFlash（SIM时间112s），14:03:10.467 MAVROS心跳超时。较晚的断连不是最初冻结的原因。前轮旧算法用 EKF速度与 UWB小位移比较误报已有回放证据；新冻结回归测试保留。

前轮只读快照：Gazebo pause=false，clock推进、ModelStates/Clock约780Hz、range25Hz；FC heartbeat/Rx停止，world_pose/IMU/velocity不再更新，localization_status继续报告FC_LOST。arducopter活着且CPU约88%，UDP9003接收队列6400B、TCP5760接收队列755703B。此为历史快照，不宣称是本轮新测量。

## 已确认

world_pose 和 ExternalNav 输出受 health 门禁控制；FC_LOST/IMU陈旧后停止发布，状态话题继续发布。ABORT本身不直接关掉 world_estimator 发布器。Gazebo独立更新不证明飞控仿真时钟前进，进程存活也不证明MAVLink处理正常。

本机 ArduPilot `libraries/SITL/SIM_Gazebo.cpp:77` 的 recv_fdm 在收到正确长度FDM包前每100ms重发servo，超时只重置 last_timestamp。时间推进依赖包timestamp差值（约120行）；同timestamp不推进，负差只加1µs。136行 drain_sockets 持续读到空，没有每轮包数上限。162行先recv_fdm，再time_advance，再drain。

本机 Gazebo插件 `~/github/ardupilot_gazebo/src/ArduPilotPlugin.cc:1049` ReceiveMotorCommand：连接后最长等servo1秒，离线1ms；1079行循环排空servo队列；1100起累积超时后重置电机控制。主机的 ROS ModelStates 正常不能证实该FDM UDP链路持续送出正确长度、递增时间戳的包。

## 推测（未定位到栈）

FDM收包等待、重复/回退timestamp、持续排空导致饥饿、其他调度/锁/计算忙循环都可能造成停滞。高CPU与队列积压仅支持“处理未正常推进”，不能据此认定卡在recv或drain，也不能断言LAND导致底层bug。没有当前线程栈和逐包证据，不对 ArduPilot 做猜测性修补。

## 采集与下一步

增强 `tools/indoor_nav_freeze_status.py`：默认5秒兼容原用法；`--duration 300 --interval 5` 增加长时检查点，记录各话题计数、最后消息、wall接收年龄、进程CPU/状态和TCP/UDP队列。等待使用墙钟，不因仿真时钟停止而无限等待。全部只读，不发布setpoint、不调用模式/解锁/复位/参数写入。CPU百分比来自ps累计值，不冒充每个采样窗的瞬时CPU。

经操作员另行批准重启且人工确认落地解除解锁后：一起加载新的mock/estimator/supervisor，先保持adapter=false，捕获5分钟检查点、ROS日志和DataFlash；再单独测试adapter=true。核对heartbeat、clock、world_pose和FDM时间推进。发生停滞时先保存该只读快照。

若仍不能定位，向操作员说明 gdb/strace 可能暂停/改变时序，得到单独许可后在确认未解锁时采样所有线程栈，并短时捕获9002/9003的包长/间隔/timestamp，与MAVLink队列对齐。当前没有该许可，因此未执行。后续根据实际栈决定是否加有限排空预算、异常timestamp保护或修复另一阻塞点，不能现在预设根因。

状态：world_pose停止的门禁原因已确认；MAVROS失联为较晚症状；SITL内部停滞根因仍待未解锁复现及栈/包证据。
