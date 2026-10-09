# POSSIBLE_UWB_FREEZE 排查与修复（2026-10-09）

本次保留工作树已有修改；未 ARM、TAKEOFF、调用 mission/start、改模式、修改飞控参数或实机 YAML。操作员说明仿真是上次实验后保留的会话，本次没有重启它。在线采样来自仍运行的旧版节点，源码修复尚未在该会话中加载。

## 本次飞行证据

ROS 会话 `ce38067c-c3a6-11f1-bc59-710cf840f56d`，日志位于 `~/.ros/log/`。DataFlash 为 `~/github/ardupilot/Tools/autotest/logs/00000343.BIN`；MAVProxy 的 `mav.tlog` 与之对应。`.sitl_runtime/logs/` 文件修改时间为 10 月 8 日，不用作本次实验依据。

| 时间（Asia/Shanghai） | 证据 |
|---|---|
| 14:01:12.405 | safety_supervisor 日志：profile=sitl，XY=[-1.1,1.1]，freeze enabled，速度阈值 0.20 m/s、位移阈值 0.015 m、窗口 1.0 s |
| 14:01:23.684 | world_estimator 锁定 world→nav yaw=102.81°，Tag 偏差 (0.040,-0.029) |
| 14:02:58.567 | safety_supervisor：ABORT / POSSIBLE_UWB_FREEZE |
| 14:02:58.569 | LAND 请求返回 mode_sent=True；DataFlash MODE 记录进入 LAND |
| 14:03:00.338–.349 | tlog 中 FC/MAVROS/GCS 的最后一批报文；DataFlash 止于 TimeUS≈112.021 s |
| 14:03:00.840 | Mock 首次警告缺少新仿真真值；14:03:05.880 再次警告 |
| 14:03:10.467 | MAVROS：HEARTBEAT timed out |

DataFlash TimeUS 109.1–110.19 s（约触发前一秒）里：SIM2 位置变化约 **7.81 mm**，速度 **0.0033–0.0139 m/s**；VISP 位置变化约 **7.31 mm**；XKF1 core 0 速度 **0.2016–0.2108 m/s**。视觉位置和仿真真值同时接近静止，不能据 EKF 速度单独认定 UWB 冻结。实际事件的逐 tick world_pose/速度没有 rosbag，不能恢复每一个回调；但已保存 33 条本次 VISP 及邻近 SIM2/XKF1 样本到 `tests/fixtures/freeze_343_window.json`，回放**能触发旧算法，新算法不触发冻结**。

因此本次有明确的**误报路径证据**：定位位置与实际飞机均近乎静止，EKF 速度仍超过冻结阈值。Gazebo 后续暂时断流和 MAVROS 心跳超时发生得更晚，不能倒推为最初冻结原因。

## 原检测到底用了什么

- safety_supervisor 的位置是 estimator 发布的 **机体 world_pose XY**，已减地面 Tag 偏差和姿态旋转后的 Tag 安装偏移；不是原始 UWB，也不是 Gazebo 真值。
- 运动证据是 `/mavros/local_position/velocity_local` 的 EKF 水平速度模长。它可能受到同一 ExternalNav 观测影响，不独立。
- timer 每 0.05 秒检查一次，速度≥0.20 m/s 且距固定位置锚点≤0.015 m 持续>1.0 s 就触发；没有保留窗口轨迹，也没有要求窗口中出现新的 UWB 观测。
- world、velocity 等 freshness 使用 **ROS 接收时刻**。忽略 header 时刻会把延迟或重复旧消息当新数据。检查提前返回时，旧冻结计时器也可能保留并跨过数据中断。
- 旧算法仅用速度模长，旋转不改变模长，所以本次误报**不能归因于 ENU/world 的轴混用**。预测围栏另有逆旋转；本次新代码显式检查 frame，并记录 nav/world 两套速度。
- estimator 中旧 `freeze_first/freeze_position/freeze_reason` 字段仅更新，没有参与 health 或停止发布，本次移除这种误导性的死状态。真正冻结保护仍在 supervisor。

## 本次实现

1. estimator 在 UwbGate 接受实际 UWB 包时发布 `/indoor_nav/accepted_uwb` (`PointStamped`)；header 为接受时的 ROS 时间，frame=`uwb_anchor`，XY 是未做机体姿态补偿的 Tag 原始坐标。这样姿态/安装杆臂导致的 world_pose 微小移动不会掩盖 Tag 数据冻结。
2. `FreezeWindow` 仅按不同 accepted_uwb 时间戳建立有界滚动窗口；重复 timer tick 不增加样本，数据间断>0.30 s、时间倒退或源变更重建窗口。计算整个窗口 XY 包络，往返运动不会因为终点接近起点被误判静止。
3. **SITL** 独立订阅 Gazebo 模型真值，依据同一窗口真值位置的路径/平均速度和变化范围确认实际运动；不再仅凭 EKF 速度。真值缺失时拒绝安全检查，**不静默退回 EKF 速度**。物体暂时静止而 EKF 速度偏高标记 `STATIONARY_WITH_EKF_VELOCITY_DISAGREEMENT`，不伪称 UWB 冻结。
4. 保留窗口 1.0 s、位置 epsilon=0.015 m、运动阈值 0.20 m/s、输入年龄 0.30 s，未放宽阈值或关闭保护。真值移动而 UWB 恒定仍触发 POSSIBLE_UWB_FREEZE，仍停止 setpoint、锁存 ABORT，并依原条件请求 LAND。
5. 单独区分 accepted UWB 停止、world_pose 过期、速度过期/错误 frame、Gazebo 缺模型/NaN、仿真时钟停滞/倒退。新鲜 Gazebo pose 不动而持续 twist 表示运动，报 `GAZEBO_POSE_TWIST_INCONSISTENT` 并拒绝安全检查，不当作正常悬停。时钟倒退锁存到未解锁 reset（重置仿真通常也需重新标定）。
6. 对现有实机 profile，不增加 Gazebo 依赖；若操作员原本启用了冻结检测，保留明确标注为 `EKF_VELOCITY_HEURISTIC` 的保护。它仍不是独立运动证明，本次 SITL 修复不能视为实机独立冻结检测已验收。未改实机配置。
7. IMU/Range 检查接收年龄和 header 年龄，拒绝旧消息重放及未来时间；UWB Linktrack 消息没有标准 Header，设备 local_time/system_time 仅记录，不能未经同步当成 ROS 时间。Gazebo ModelStates 也无 Header，记录接收年龄，并监视独立 `/clock` 进度。安全 watchdog 明确要求 `/use_sim_time=false`，避免暂停仿真同时暂停安全 timer 和数据年龄。

## 新诊断与捕获

| 话题 | 内容 |
|---|---|
| `/indoor_nav/abort_snapshot` | latched、触发瞬间的完整窗口：每样本原始 UWB、world XYZ、nav/world 速度、Gazebo XY/twist、ROS 样本时间、各消息接收/header 年龄、阈值、分类、触发条件和 FC/任务状态。后续停更不会覆盖该事件 |
| `/indoor_nav/safety_diagnostics` | 持续的窗口摘要；完整 samples 仅在事件快照/分类转移日志，避免反复广播大窗口 |
| `/indoor_nav/localization_diagnostics` | 持续报告 FC/IMU/Range/UWB 年龄、设备时间、原始/接受坐标、拒绝原因、定位 health、最近一次 world_pose 发布时间；状态转移落入 ROS 日志 |
| `/indoor_nav/sitl/mock_diagnostics` | 真值位置与年龄、clock 进度、模型来源、队列长度、freeze/drop_all 注入状态及停发原因；状态转移记录日志 |

源不可用时清空计时以免跨断流误报，并在首次故障事件中附上 previous_window。Mock 对缺模型/非有限值立即使真值失效、清空延迟队列；钟停或真值断流时不再 flush 旧队列。静止但持续新鲜的真值仍正常发布，freeze 注入只固定 UWB，drop_all 注入仅停 UWB，Range 继续更新。

```bash
source /opt/ros/noetic/setup.bash
source ~/github/indoor_nav_ws/devel/setup.bash
# 只读五秒捕获，含源数据/年龄、Gazebo pause、进程及 UDP/TCP 队列
python3 tools/indoor_nav_freeze_status.py > /tmp/freeze_status.json
rostopic echo -n1 /indoor_nav/abort_snapshot
rostopic echo -n1 /indoor_nav/localization_diagnostics
rostopic echo -n1 /indoor_nav/sitl/mock_diagnostics
# 下次实验开始前记录；这些命令不会 ARM 或起飞
rosbag record -O /tmp/indoor_freeze.bag /gazebo/model_states /clock /nlink_linktrack_nodeframe2 /indoor_nav/accepted_uwb /indoor_nav/world_pose /mavros/local_position/velocity_local /mavros/state /mavros/imu/data /indoor_nav/localization_status /indoor_nav/safety_status /indoor_nav/abort_snapshot /indoor_nav/localization_diagnostics /indoor_nav/sitl/mock_diagnostics /diagnostics
```

部署需要在操作员确认仿真已落地、未解锁之后，协调重启 indoor 节点（estimator、mock、supervisor 一起更新），不能只替换 supervisor 而继续用缺少 accepted_uwb 的旧 estimator。本次未重启旧会话、未做新的飞行验证。

## world_pose 停更与 MAVROS 断连

本次在线只读快照保存在 `docs/evidence/freeze_live_20261009.json`，采样期间：

- Gazebo `pause=false`；5 秒内约 3900 条 ModelStates 和 Clock，仿真时钟持续前进。
- Mock Range 约 25 Hz，localization_status 约 30 Hz；UWB/Range 年龄均为数十毫秒，**reason=FC_LOST**。
- world_pose、FC IMU、FC velocity 无新消息；MAVROS 接收速率 0，心跳频率 0；state 为旧的 disconnected 消息。
- 因 estimator.health 的 FC_LOST 分支返回 false，publish 主动停止 world_pose 与 ExternalNav。ABORT 不直接调用或关闭 estimator；持续发布陈旧姿态下的“健康位姿”会掩盖失联，因此没有绕过该门禁。
- ArduCopter 进程仍在，平均 CPU 约 88%；9003 接收队列约 6400 字节，飞控 TCP 5760 接收队列约 755703 字节，连接仍建立但 FC 无新心跳。结合日志停止于 FC 时间 112 秒，支持**飞控/仿真处理停滞、通信未被正常消费**，不是 ROS world_pose 节点已退出或 Gazebo 当前被暂停。

用户已说明这是实验后的保留会话，进程存活本身不是异常。上述快照说明的是**当前通信状态**，不是重新发生了一次飞行故障。系统诊断中的 CPU load/模式/电机健康等可能是断连前缓存值，不能拿 disconnected 后的 armed=false 当成实际飞控已解锁/未解锁的可信证明。

`SIM_Gazebo.cpp::recv_fdm` 在 FDM 缺失时等待重发，仿真输入停顿可以阻塞推进；但当前还有 TCP 队列积压和高 CPU，**尚未定位到飞控具体阻塞/自旋堆栈**。没有附加调试器、暂停或重启用户保留的仿真，不能声称已修复外部 ArduPilot/Gazebo 的停滞。建议下一次在未解锁复现中同时保存 Gazebo/SITL 控制台及上述捕获；现有数据不足以确定最初 14:03:00 暂时断流的内部原因。

另有可复现的 EKF 质量问题：本次全部 VISP 的 PErr=100 m。当前 ROS Odometry Z 方差为 1e3，而本机 Copter 4.5.7 `libraries/GCS_MAVLink/GCS_Common.cpp:3682` 将 XYZ 对角方差平方求和后开立方，`AP_VisualOdom_MAV.cpp:33` 上限钳到 100。大 Z 方差会同时降低 XY 位置观测权重，不是“只让 EKF 忽略 Z”。这与速度估计偏差相符，是重要待修问题；本次没有擅自降低方差、修改 VISO_POS 或改实机参数。消除冻结误报不代表 EKF 权重问题已解决，暂不据此授权再次航点飞行。

## 测试

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
python3 -m compileall -q src/indoor_nav/scripts tools/indoor_nav_freeze_status.py tests
git diff --check
```

验证：主测试 33/33、core 9/9，通过且未跳过；catkin_make、Python 编译、diff 检查通过。涵盖本次 DataFlash 回放（旧触发、新不触发）、真冻结触发、正常移动/悬停、往返运动、重复样本、断流/时间倒退、旋转后速度方向、窗口快照持久保留、停止 setpoint 与 LAND 门禁、world_pose/速度陈旧、Gazebo 错模型/NaN/clock 停滞与倒退、pose/twist 矛盾、freeze/drop_all 注入、FC_LOST 与 Range 旧消息导致停发。

这些测试调用实际节点回调和 ROS 生成消息，但 transport/timer/clock 使用桩；没有自动 ARM、起飞或在在线仿真中注入故障。
