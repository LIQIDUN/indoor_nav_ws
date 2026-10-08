# indoor_nav — Nooploop + MTF-01P + ArduPilot ROS1 V1

本包实现《室内 UWB 定位与 ArduPilot 航点飞行系统 V1 方案》的**功能级软件原型**。仅供 **ArduPilot SITL、拆桨及隔离实验场**逐级验证；未通过实机测试，**不是已经验证可安全自主飞行的成品**。

## 0. 安全优先的默认设置

- **不会自动 ARM、起飞、切 Guided、自动改 FC 参数或关闭 ArduPilot failsafe。**
- 实机 `external_nav_enabled=false`，不向 FC 发外部位置，直到你明确配置打开。
- `anchor_map_verified=false`、`operator_verified_fixed_ekf_origin=false`，未进行实地校准前无法 READY。
- Mission 只有收到 `/indoor_nav/mission/start` 显式服务调用，且飞机已由飞手手动 ARM、升空、切 GUIDED，才开始送航点。
- `mission_manager` **只发布** `/indoor_nav/desired_setpoint`；只有 `safety_supervisor` 可向 `/mavros/setpoint_position/local` 转发合规目标。
- 传感器/定位/任务失效时安全监督会尝试请求 `LAND`，但**此请求不等于物理安全保证**；遥控器必须有独立的 ALT_HOLD/STABILIZE 接管，并测试飞控自带 RC、EKF、电池 failsafe。
- SITL 配置会明确将部分检查设为模拟环境专用，不可照搬到实机。

## 1. 目录与职责

```text
indoor_nav/
├── scripts/
│   ├── world_estimator.py     # Node_Frame2; Ground Tag; 固定 world; ExternalNav XY
│   ├── mission_manager.py     # YAML 航点、目标限速、显式 start/stop
│   ├── safety_supervisor.py   # 安全门、围栏、参数校验、LAND 请求
│   └── mock_uwb_node.py      # SITL 用，模拟同样的 nlink_parser ROS 消息
├── src/indoor_nav/core.py     # 无 ROS 依赖的坐标和 UWB 检查数学
├── msg/                      # LocalizationStatus / SafetyStatus
├── config/                   # localization / mission / safety / sitl / FC expected
├── launch/                   # real 和 SITL 两种运行入口
├── test/                     # 无 ROS 依赖的单元测试
└── docs/                     # 现场 SOP 和接口注意事项
```

依赖包：ROS1 Noetic (Ubuntu 20.04)、`mavros`、`mavros_extras`（odometry 插件）、`nlink_parser`、`tf2_ros`、`std_srvs`、`gazebo_msgs`（仅 SITL）；飞控 ArduPilot Copter；地面 Console 使用 `Anchor_Frame0`、机载 Tag 用 `Node_Frame2`。**不要修改官方 Nooploop 包。**

```bash
cd ~/catkin_ws/src
git clone https://github.com/nooploop-dev/nlink_parser.git
# 解压 ZIP 后将 indoor_nav/ 放在同一级 src/ 下
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> `indoor_nav` 不替换已存在的 `all_sensors.launch` / 声学节点 / rosbag。保留已有 MAVROS，不能同时运行两个连接同一 FC 的 MAVROS 实例。

## 2. 坐标定义（绝对重要）

- `world_lab`: 永久实验室坐标，X/Y/Z 为右手 ENU-style，Z=离地高度。
- `odom`: MAVROS local ENU navigation 坐标；其原点不是“启动后每次随意重置的 world_lab”。
- `T_nav_world`: `(world_lab XY) -> (odom XY)` 2D 刚体变换，**只在拆桨/起飞前做一次标定，ARM 后不更新**。
- `world_to_nav_yaw`：飞机机头先物理对准 `world_lab +X`，标定时读取 FC 的当前 ENU yaw 作为坐标旋转。随后飞机转动**不会**改变此旋转。
- `nav_xy_at_world_origin`：世界原点在 EKF 本地位置系中的固定 XY。**如果 EKF 本地原点不在约定位置，这里需要经实测校准，不能默认填 0 就视为正确。**
- 飞行航点从 world_lab 转到 odom。两个方向必须手持/拆桨测试：向 +X/+Y 移动时 EKF 对应方向正确。
- `z_offset_m`：标定时计算 `(FC local Z - laser-derived world Z)`，用于把 world 高度航点转换到 MAVROS local Z；飞行中冻结。
- ExternalNav ODOMETRY **Z 始终使用 FC 当前 local Z**、不是 UWB Z，也不是任意的 0。特意规避部分 ArduPilot 混合源版本中外部 ODOMETRY Z 导致 EKF 高度突然改变的问题。必须在目标固件的 SITL 验证这一数据源组合。
- ExternalNav ODOMETRY quaternion 使用飞控当前姿态（V1 compass yaw），**不是** UWB 输出的 angle/quaternion；这些字段对多数 LinkTrack 型号不是有效 yaw。
- 按 MAVROS 要求静态转换：`odom_ned <- odom`、`base_link_frd <- base_link`。launch 内已经提供静态 TF，可在与已有 TF 冲突时关闭该选项。

## 3. 真实 Nooploop 接线/协议

Nooploop 官网 ROS 包：<https://github.com/nooploop-dev/nlink_parser>

| 使用端 | 输出协议 | 话题 | 消息 |
|---|---|---|---|
| Aircraft T0 | `Node_Frame2` | `/nlink_linktrack_nodeframe2` | `nlink_parser/LinktrackNodeframe2` |
| Ground Console C0 | `Anchor_Frame0` | `/nlink_linktrack_anchorframe0` | `nlink_parser/LinktrackAnchorframe0` |

`LinktrackNodeframe2.pos_3d` 是本机位置，`eop_3d` 供健康门使用；其中 `nodes[]` 只有远端 `role/id/dis/fp_rssi/rx_rssi`，不是远端完整位置。地面 Console 的 `Anchor_Frame0.nodes[].pos_3d` 可用于起飞前 Ground Tag T1 修正。Ground Console 可以在地面 PC 上通过同一 ROS master 的网络发布，或者把校准结果离线保存；**机载飞行时不依赖实时地面 Wi-Fi 链路**。

修改 `config/localization.yaml` 中：

- Aircraft Tag 和 Ground Tag 真实 ID；
- T1 的**真实已测量**物理 XY；
- `anchor_map_verified`（明确实测后才打开）；
- `range_topic`：必须用 `rostopic list | grep -Ei 'range|distance'` / `rostopic type` 确认是 **sensor_msgs/Range** 且为 MTF-01P；
- `nav_xy_at_world_origin`；
- MTF 激光相对机体参考点高度 `range_to_body_z_m`；
- `body_to_tag_flu_m`：机体参考点→UWB 天线的三维 FLU 外参（实时用 FC 姿态扣除安装偏移）；
- 软件围栏数值必须按**真实机体外轮廓/螺旋桨半径 + 制动距离 + 定位误差**重新计算，示例 0.4 m 缓冲不保证足够；
- `world_to_nav_yaw` 是标定得到、不是在 YAML 里任意给一个不会被验证的角度。

初版校准要求：在**完全拆桨**的状态，飞机在固定起始姿态、机头沿世界 +X，Ground Tag 静止超过约 8 s，飞控本地姿态及激光数据有效，然后调用：

```bash
rosservice call /indoor_nav/calibrate
```

如果 Console/地面参考无法连到机载 ROS，先临时把 Ground Tag startup calibration 置为 disabled，但必须人工确认固定 Anchor Map 及变换的一致性；不能一边关闭参考一边声称实现了跨实验日自动漂移修正。

## 4. MTF-01P 直接接飞控

**不另写 MTF 驱动。** MicoAir 官方串口 `Mavlink_apm` 或 AUTO 协议，115200 baud。飞控配置需按 **实际 ArduPilot 固件版本**确认（4.7+ 中 `RNGFND1_MAX` 与旧 `RNGFND1_MAX_CM` 命名不同）。

典型融合：

```text
EK3_SRC1_POSXY = 6  # ExternalNav
EK3_SRC1_POSZ  = 2  # RangeFinder，仅适用于平坦、无遮挡地面
EK3_SRC1_VELXY = 5  # OpticalFlow
EK3_SRC1_VELZ  = 0
EK3_SRC1_YAW   = 1  # V1 compass, 必须检查室内电磁环境
FLOW_TYPE      = 5
RNGFND1_TYPE   = 10
RNGFND1_ORIENT = 25
```

飞控 RangeFinder 和 Optical Flow 各自数据的刷新率、健康度必须检查。`RNGFND1_ORIENT` 以官方安装方向为准；**光流正负轴接反会形成正反馈导致失控，拆桨验证光流方向是硬性步骤**。激光检测的是到被照射表面的距离，地面不能有桌椅/箱体或其他障碍物干扰。

注意：`range_topic` 在 MAVROS 中不一定自动存在。若只有 MAVLink `DISTANCE_SENSOR` 但没有 `sensor_msgs/Range`，必须配置 MAVROS distance_sensor 插件或另加一个安全的适配节点；**当前包不把 `/mavros/local_position/pose.z` 伪装成物理激光高度**。没有真实 range topic 则 READY 失败，这是有意设计。

## 5. ArduPilot ExternalNav 输入

MAVROS 主题 `/mavros/odometry/out`，消息 `nav_msgs/Odometry`。默认 `external_nav_enabled=false`。打开前需确认 MAVROS odometry 插件订阅且有正确静态 TF：

```bash
rostopic info /mavros/odometry/out
rosrun tf tf_echo odom_ned odom
rosrun tf tf_echo base_link_frd base_link
```

`world_estimator` 还输出：

```text
/indoor_nav/world_pose         geometry_msgs/PoseStamped  (world_lab; laser world z)
/indoor_nav/ap_odom            nav_msgs/Odometry         (odom; pre-MAVROS)
/indoor_nav/nav_from_world     geometry_msgs/TransformStamped (latched)
/indoor_nav/z_offset_m         std_msgs/Float64          (latched)
/indoor_nav/localization_status indoor_nav/LocalizationStatus
```

**必须检查** ArduPilot 对 ODOMETRY 的 Z/姿态/速度和 covariance 实际如何融合，包括“只设 POSXY=6 也可能在首次 ExternalNav 输入时发生 Z 跳变”的已报告问题（参见 `docs/REFERENCES.md`）。仅将 XY 设置为 ExternalNav 并不意味着 MAVLink 消息里 Z 字段完全被忽略。

## 6. 任务 YAML / 三层安全门

`config/mission.yaml` 提供 **以 world_lab 米为单位**的航点、yaw 和 hold。程序不自动起飞；飞手先在遥控器控制下起飞并进入 Guided，确保位于软件安全空间内，再：

```bash
rosservice call /indoor_nav/safety/acknowledge
# 注意：这条必须在 DISARM 时执行；一般在起飞前完成
# 起飞 + 切 GUIDED 后：
rosservice call /indoor_nav/mission/start
```

如果任务过程中需要终止，飞手**优先通过遥控器切 ALT_HOLD / STABILIZE**，然后再调用 `/indoor_nav/mission/stop`；如果只调用停止服务且仍保持 GUIDED，安全监督将尝试 LAND。

目标转发路径：

```text
YAML → mission_manager (/indoor_nav/desired_setpoint)
   → safety_supervisor → /mavros/setpoint_position/local → ArduPilot Guided
```

到点需要 XY、Z、yaw 同时进入容差，连续满足 `stable_time_s`，再额外保持 `hold_s`。完成航点后保持最后目标，直到飞手切模式接管；**不自动 LAND**。本版的目标插值只是**位置限速**，不是严格加速度/jerk 轨迹规划，`max_accel` 应在 ArduPilot 控制参数中独立限幅核查。

### 安全监督

- 启动时比较 `/mavros/param/get` 返回的 FC 配置与 `config/ardupilot_expected.yaml`。不一致则不能 READY，不会自动改飞控。
- 正常实机需将 `operator_verified_fixed_ekf_origin=true`，前提是飞手已核实固定 EKF Origin、航向及坐标系。
- 坐标锁定、UWB 超时和 jump、激光超时、地面围栏、越界趋势（速度短期预测）、非法航点、目标超时均阻断航点转发。预测只依赖 FC 速度估计，不是障碍物探测或碰撞保证。
- 可选 heuristic freeze 检查默认为关闭：FC 速度不是独立测量，容易误报，必须实测验证后开启。
- 长时间 UWB 失效时不冻结最后状态再冒充正常；本节点直接停止 healthy ODOMETRY 发布。监督检测后尝试请求 LAND。
- **机载电脑突然死机**时监督本身也会停止：因此 FC 原生 EKF/RC/电池 failsafe 必须单独设置并完成故障注入，不能仅信任上层 Supervisor。

## 7. 功能级 SITL

启动独立 `ArduPilot SITL + Gazebo11 + MAVROS`（你们现有环境），且必须确认 Gazebo 独立真值话题（默认 `/gazebo/model_states`，模型名默认 `iris`）；如模型名不同修改 `config/sitl.yaml`。**mock 不允许使用 `/mavros/local_position/pose` 作为真值输入**，因为这样会形成自反馈。

SITL 模式启动：

```bash
roslaunch indoor_nav indoor_nav_sitl.launch
```

这个 launch 专为 SITL 显式设置：

- `anchor_map_verified=true`；
- `operator_verified_fixed_ekf_origin=true`；
- `check_fc_params=false`（仅首次 Mock 链路验证使用；后续应打开确认 FC 参数）；
- `freeze_detection/enabled=true`（SITL 故障注入使用，实机默认关闭直到验证）；
- `external_nav_enabled=true`。

SITL 启动文件把 ROS 侧的 `height/range_topic` 改为 `/indoor_nav/sitl/mock_range`，并采用示例安装偏移 `Range = Gazebo body_Z + 0.15 m`、`range_to_body_z_m = -0.15 m`。这个 Range 来自**独立 Gazebo 真值**，仅作为上层安全/航点所用的激光观测模拟值，并**不会送入 ArduPilot**。仍然必须**在 ArduPilot SITL 内**按官方支持的配置单独启用模拟 RangeFinder / Flow，且确认 FC EKF 真的在融合 Range/Flow；本包不自己修改 AP 固件参数。首先做坐标正反、Z 无突变检查，再闭环。

模拟 Ground Tag 静置约 8 s 后，确保 SITL 飞机假定机头沿 `world_lab +X`，然后：

```bash
rosservice call /indoor_nav/calibrate
rosservice call /indoor_nav/safety/acknowledge
```

SITL 可动态注入故障：

```bash
rosparam set /indoor_nav/mock/uwb_sim/noise_std_xy_m 0.10
rosparam set /indoor_nav/mock/uwb_sim/jump_now true
rosparam set /indoor_nav/mock/uwb_sim/freeze true
rosparam set /indoor_nav/mock/uwb_sim/drop_all true
```

每次注入故障，确认 `/indoor_nav/localization_status` / `/indoor_nav/safety_status`，并检查飞控实际 mode 和 EKF 日志，不只检查 ROS 层输出。

## 8. 需要实机确定的值

1. Nooploop 实际型号、两颗 Tag ID、Anchor Map 真实几何；
2. Ground Console 数据怎么传至 ROS 主 master，以及 UWB 实际更新率；
3. MAVROS 是否发布 MTF-01P 的 `sensor_msgs/Range`，真实话题、Sensor Frame 与倾斜修正；
4. MTF-01P 的 OpticalFlow 方向和 RNGFND 端口配置；
5. 固定 EKF global origin 与实验室 `world_lab` XY 对应的 nav origin；
6. FC 本地高度相对物理地板的初始差值和不同开机高度可重复性；
7. FC yaw 校准、磁干扰下 yaw 是否足够稳定；
8. 实机 ArduPilot 版本的 `ODOMETRY` 混合高度/速度行为；
9. 接收机手动模式切换和不依赖 companion 的 failsafe。

## 9. 测试顺序

```text
源码 unittest / py_compile
→ 功能级 SITL 静态坐标检查
→ SITL 地面校准
→ SITL ExternalNav XY + simulated Z/Flow
→ SITL 单航点 + 方形
→ SITL 跳点/断流/freeze/越界故障注入
→ 拆桨实机 +X/+Y/+Z 与 yaw 旋转
→ 拆桨检查 RC 模式与 failsafe
→ 安全固定装桨验证
→ 低空手动 AltHold
→ ExternalNav 悬停
→ 0.3m 单航点
→ 0.5m / 1m 方形
```

## 10. 关联文档和原始设计

文档基于用户项目的《室内 UWB 定位与 ArduPilot 航点飞行系统 V1 方案》，本包没有改动原项目声学采集系统。所有未经过目标硬件实测的参数、话题和坐标偏差保持为待验证项。
