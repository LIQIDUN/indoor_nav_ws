# V0 仓库审计（2026-10-26，Asia/Shanghai）

## 保护与身份

- 工作区 `/home/liqidun/github/indoor_nav_ws`，ROS1 catkin；起始分支 `master`，HEAD `92f6e724d4ee9ba8c2a348d4ace108e561a4ef30`。
- 新分支 `feat/arducopter-v0-relative-waypoints` 从上述提交建立；没有 reset、stash、clean、覆盖或远程 push。
- 起始仅四个未跟踪用户输入：`docs/ArduCopter_V0相对航点_完整项目设计与验收文档.md`、`docs/Codex_开发提示词_ArduCopter_V0相对航点.md`、`docs/README_使用说明.md`、`docs/mission_v0_copter.yaml`。它们不属于本次代码提交，不改字节。
- `evidence/audit/initial.txt` 保存要求的 pwd/status/branch/log/find；`tracked.sha256` 和 `user_inputs.sha256` 为写代码前快照。`git log` 为提交最终身份的权威来源，不在提交内自引用 SHA。
- 没有适用于本工作区的 AGENTS.md。遵守最小侵入；`CODEX_INDOOR.md` 为原 ExternalNav 任务背景，新任务的 GPS/V0 边界优先。

## 真实结构及历史

- `src/indoor_nav/scripts/{mission_manager,safety_supervisor,world_estimator,mock_uwb_node}.py`；数学/速度逻辑在 `src/indoor_nav/src/indoor_nav/`。
- `config/{mission,safety,localization,uwb_velocity,...}.yaml`；旧 launch 为 `indoor_nav.launch` / `indoor_nav_sitl.launch`；测试在根 `tests/` 与包内 `test/`。
- 根目录没有 `evidence/`；真实历史在 **`docs/evidence/`**。`docs/evidence/sitl_flight/run_06/` 存在 bag、DataFlash、tlog、events、telemetry、summary、analysis。
- `run_06/summary.json` 标记 PASSED；时间差为 53.1284935 s；航点和真值分析存在。`git log -- docs/evidence/sitl_flight/run_06` 指向 `3b3073c`，目前 HEAD 后续为 `92f6e72`。历史记录 10/10、ExternalNav，不等于本轮重新飞行，更不是 Fake GPS 验证。
- 旧 mission 无自动 ARM/起飞，要求已 ARM/GUIDED、标定/world_pose、安全 READY；在 world 坐标插值目标，交由 supervisor 唯一转发。
- 旧 supervisor 负责校准、ACK、world_lab 围栏、UWB freeze、速度 frame、故障 LAND，直接拥有 `/mavros/setpoint_position/local`。**不能与 V0 同时运行**。
- 旧 `world_estimator` 向 `/mavros/odometry/out` 注入 ExternalNav。V0 不启动、不复用该数据源；仅独立复用包安装与 ROS 环境。V0 数学核心独立，避免改变历史行为。

## 软件与入口

- Ubuntu 环境 ROS Noetic；安装 `ros-noetic-roscpp 1.17.4`、`ros-noetic-mavros 1.20.1`、Gazebo Classic `11.15.1`。
- `/home/liqidun/github/ardupilot/ArduCopter/version.h` 为 **4.5.7**，源码 HEAD `2a3dc4b7bf`（describe 包含上游标签 AION，不用标签猜固件）。动态固件身份由 `/mavros/vehicle_info_get` 再核验并保存。
- 旧 GUI `tools/start_indoor_sitl.sh`：ROS11311、Gazebo11345、MAVROS14552；旧隔离 `tools/sitl_flight_instance.sh`：ROS11321、Gazebo11355、MAVROS14572，仍占 TCP5760、UDP9002/9003。
- 新 V0 `tools/v0_sitl_instance.sh --sitl`：ROS11421、Gazebo11455、MAVROS14672，仍与旧仿真争用 TCP5760、UDP9002/9003，**先独占端口检查，不允许同时启动**。每次新建 `.sitl_runtime/v0/<timestamp-pid>`，不复制/覆盖旧 EEPROM；仅在副本改 Gazebo step 为 0.001 s。
- 初次沙箱内 ps/netlink 可见性有限，不能以空输出宣称主机无进程。实际新 SITL 在授权的主机进程/套接字上下文中成功通过绑定独占检查。
- 最终主机级审计发现本轮之前的MAVROS PID2391064（2026-10-14启动）和3263919（2026-10-15启动），均有至旧11311的CLOSE_WAIT及UDP14550；未见它们占用本轮11421/14672/FDM端口，也未见tty句柄。未停止/修改这些用户进程。因此“单一MAVROS”只指本轮隔离ROS图，**不是主机全局只有一份MAVROS**。最终本轮所有进程/端口已释放，保留原有两进程。

## MAVROS 接口现场差异

- `/mavros/state`: `mavros_msgs/State`；pose: `geometry_msgs/PoseStamped`；`velocity_local`: `geometry_msgs/TwistStamped`；`/mavros/estimator_status`: `mavros_msgs/EstimatorStatus`。
- 本机 1.20.1 的 **velocity_local 数据为 ENU，却沿用 odom.child_frame_id=`base_link` 标签**；不能按标签再次转机体系，也不能订阅 velocity_body 冒充 local。官方插件源码快照和运行 echo 在审计证据中。V0 只在这一指定话题允许 `map` / `base_link`，其他 frame 拒绝。pose/目标始终要求 `map`。
- 首次 V0 单点因原严格 `map` 速度标签检查在 START 拒绝；已完整保留失败，不绕过其它门槛。
- 第二次因通用msg230未出数而EKF_STALE拒绝；追溯旧runner及目标Copter源码，真实为msg193 EKF_STATUS_REPORT。新增仅接收既有MAVROS转发的固定报文解码/flags/ratio检查，不是MAVLink直连或GPS注入。
- 仅支持经审计的 Copter4.5.7：`WPNAV_SPEED/UP/DN` 为 cm/s，`WPNAV_ACCEL/ACCEL_Z` 为 cm/s²；Guided `pva_control_start` 从 WPNav 读这些限制。未知固件/新 `WP_SPD` 不猜测单位，拒绝 START。生产节点不写参数。

## 采集与 Fake GPS 缺口

- 开发机找到 `/home/liqidun/github/ros_sensor/src/rs485_driver_sound/launch/all_sensors.launch`，实际仅启用 vibration；与任务附件描述的 XVF/MAVROS/auto_bag 快照不一致。未发现可核实的机载最新 IM72/自动录包实现；**没有修改该工作区**。
- 未找到真实 UWB→Fake GPS 生成代码、设备连线/端口证据。不能推定 GPS_INPUT、HIL_GPS 或串口协议；F2 BLOCKED。详见 `FAKE_GPS_INTERFACE_GAP.md`。
- F3 只能模拟 PCM 大小/带序号的 ROS 负载与独立 rosbag；不能验证真实采样、ESC、ARM/DISARM 自动录包逻辑或机载端口独占。
