# 无人机声学实验：ArduCopter V0 相对航点系统
## 项目架构、接口定义、实现规范、SITL 验收与真机测试 SOP

- 文档版本：**V0.1 / 2026-10-26**
- 文档状态：**开发基线（设计已明确，新增 V0 代码尚待 Codex 在本地实现）**
- 软件平台：Ubuntu 20.04、ROS1 Noetic、MAVROS、ArduPilot **ArduCopter**、Python3、catkin；实际版本以本机实测为准。
- 源码入口：`~/github/indoor_nav_ws`，在新分支 `feat/arducopter-v0-relative-waypoints` 增量开发。
- 关联系统：已有 `rs485_driver_sound` 传感器采集、MAVROS 飞行数据、ARM/DISARM 自动 rosbag。
- 配套交付：`Codex_开发提示词_ArduCopter_V0相对航点.md`、`mission_v0_copter.yaml`。

> **边界声明**：本文件只定义架构、设计要求、测试门槛和参考命令，不声称已经在本轮建立了 Git 分支、修改本地源码、运行 SITL 或完成真机试飞。本地 `indoor_nav_ws` 完整源码没有附在本轮对话；后续实际接口和文件名必须由 Codex 现场检查。

---

## 1. 背景、目标与取舍

### 1.1 实验目的

本项目面向六旋翼无人机电机/桨叶声学状态监测与故障诊断。已有采集系统持续记录音频、ESC 转速、振动、姿态和飞行状态。新增相对航点的目的是**让飞机以低速、可重复、可观察的方式执行简单运动实验**，为不同动作阶段采集可对齐的声学数据。项目当前**不以厘米级绝对室内定位或世界坐标跨天固定复现为首要目标**。

最小成功场景：飞手人工起飞并稳定悬停 → 机载电脑通过 YAML 指定“前进 0.3m、右移 0.3m、停留、返回起点” → ArduCopter 在 Guided 模式执行 → 录包仍持续，离线可对应每段动作与 ESC RPM / 音频。

### 1.2 决策

| 设计点 | V0 决定 | 说明 |
|---|---|---|
| 开发位置 | 既有 `~/github/indoor_nav_ws` 新分支 | 保留历史 ExternalNav 功能 |
| 飞控类型 | ArduCopter | 不采用 QuadPlane Guided 语义 |
| 定位输入 | 原有 UWB Fake GPS 路线 | 不在本任务里重写定位链路 |
| 导航位置 | **飞控 EKF 融合后 local pose** | 相对航点不直接使用原始 UWB 坐标 |
| 机载控制接口 | **ROS1/MAVROS** | 禁止新建 pymavlink 直连 FCU |
| 任务描述 | 机载 YAML | 任务坐标、停留、容差、限制文件化 |
| 航点参考 | 一次冻结的任务起点与初始机头方向 | 不需要实验室固定世界原点 |
| 飞控控制层 | ArduCopter Guided | 机载机不接管底层姿态与电机 |
| ARM/起降 | V0 由飞手负责 | 默认禁止自动 ARM、起飞、降落 |
| 并行采集 | 共享一份 MAVROS、ROS Master | 导航与采集业务彼此独立 |
| 测试 | 先 SITL 后实机 | Fake GPS **真实接口等价**的 SITL 是准入条件 |

### 1.3 明确不做

不启用原 V1 的 `world_lab` 固定世界坐标/地面参考 Tag/ExternalNav 作为 V0 的必要前提；不修改声音采集算法、PCM 格式、传感器驱动、ESC 采集；不增加第二个飞控串口读写进程；不允许实机自动测试脚本在没有飞手授权的情况下解锁/起飞；不把位置掉线后的 Loiter/RTL 视为可靠保护。

---

## 2. 已有项目事实与需现场核查内容

### 2.1 本轮可直接核实的采集系统

项目附件 `all_sensors.launch`（注意是**上传快照，而非必然等于真机当前最新版**）显示：

1. `rs485_driver_sound/vibration_node.py`（示例串口 `/dev/ttyUSB0`），发布 `/vibration_data`；
2. `rs485_driver_sound/xvf3800_node.py`，16 kHz、6 通道音频，10 Hz 发送 1600 帧/通道的完整采样块，发布 `/sound_sensor/audio_raw`；
3. 一次 `$(find mavros)/launch/apm.launch`，默认 FCU URL `/dev/ttyACM0:115200`，`tgt_system=1`、`tgt_component=1`；
4. `rs485_driver_sound/auto_bag_recorder.py`：订阅 `/mavros/state`，ARM 时 `rosbag record`，DISARM 时 SIGINT 停止；
5. 录制 `/mavros/local_position/pose`、`/mavros/local_position/velocity`、`/mavros/esc_telemetry`、`/mavros/imu/data`、`/mavros/state`、电池、RC、XVF raw、振动等。

项目声学硬件基线还定义了一套**额外的** 6×IM72D128 近电机采集板（48 kHz / 24 bit / 6ch USB UAC，建议话题 `/motor_mic/audio_raw`），但上传的旧 `all_sensors.launch` 未启动它。因此 Codex **必须先检查机载最新版**，不得用上述较老的 launch 覆盖较新传感器功能。XVF3800 中央阵列与 IM72D128 近电机阵列应同时保留。

### 2.2 以前室内导航的历史记录

此前 `室内UWB定位与航点飞行_V1方案.md` 描述了 Nooploop LinkTrack、MTF-01P 测距/光流、Pixhawk EKF3、ExternalNav、`mission_manager`、`safety_supervisor`、Mock UWB、Gazebo/SITL 等。它属于**更完整的 V1 设计**，不是必须照搬的 V0 实现。

既有项目记录中，`~/github/indoor_nav_ws` 曾在 2026-10-09 的 `run_06` 以 **ExternalNav** 在 ArduCopter SITL 完成 10/10 航点（约 53.13 秒）；入口曾为 `SITL_FLIGHT_COVARIANCE_ADAPTER=true ./tools/sitl_flight_instance.sh`。这一成果值得保留及回归，但**不能用作 Fake GPS 已通过的证据**。

历史集成还曾遇到 `INVALID_VELOCITY_FRAME`，以及静止目标接近零位移却被旧 EKF 速度阈值误判为 UWB freeze 的情况。因此 V0 应仅复用经过甄别的通用安全逻辑，不能照抄旧 ExternalNav 必需的速度 frame、`world_lab`、calibration、freeze 规则。

### 2.3 开发前必须检查而不能猜的事实

| 必查项 | 为什么重要 | 检查方法示意 |
|---|---|---|
| 真实源码和当前 Git 状态 | 防止覆盖旧版本或用户未提交修改 | `git status; find src/indoor_nav ...` |
| 飞控固件版本与机型 | 决定 Guided、参数名和 EKF 枚举 | 飞控版本、MAVROS 连接输出、FC 参数快照 |
| 真机 Fake GPS 的具体来源 | F2 要模拟**相同入口** | 查 UWB 设备接线、程序、FC 串口、MAVLink 消息 |
| Fake GPS 是否复用 FCU 通信口 | 避免多程序重复占用 | `lsof`、FCU 端口配置、传输拓扑 |
| `local_position/pose` 的 frame、频率、时间戳 | 决定相对坐标与 health 门限 | `rostopic type/echo/hz` |
| 高度源、速度源与 yaw 源 | 影响 Guided 的真实闭环稳定性 | EK3_SRC1_*、RangeFinder/Flow/Compass |
| 机载最新版声学 launch、IM72D128 | 不丢采集通道 | 实机包路径与 source 对照 |
| RC 控制模式、失联 failsafe | 人工接管和断 GPS 处理 | 拆桨台架与 SITL fault injection |

### 2.4 对 Fake GPS 的正确认识

飞机没有真实 GNSS 也可以通过 UWB 提供一个参考坐标，转换为飞控接受的 GPS 类输入。V0 不需要**全球绝对真实经纬度**，但必须有**连续、方向正确、质量足够的飞控位置估计**。一旦当前 local pose 来自 Fake GPS 的 EKF 融合不健康，航点不得继续。

ArduPilot 的 GPS 类位置源与 ExternalNav 是不同路径。原 ExternalNav 通常使用 EKF POSXY=ExternalNav（枚举 6），V0 Fake GPS 通常使用 EKF POSXY=GPS（枚举 3），**具体值和参数名必须由目标固件实际确认**。在同一实例中不准同时盲目打开冲突的 GPS/ExternalNav 注入。

---

## 3. 总体软件和物理架构

```text
                     [Nooploop UWB 基站与机载 Tag]
                                     |
                           [已有 Fake GPS 路径]
                                     |
                           [Pixhawk / ArduCopter]
                           IMU/EKF3/Guided/RC failsafe
                                     ^
                                     | 一条现有 MAVLink FCU 链路
                                     v
                        +-------------------------+
                        |   MAVROS（唯一实例）     |
                        |   /mavros/*             |
                        +------------+------------+
                                     |
                        ROS Master（唯一实例）
                             /             \
                            /               \
    [rs485_driver_sound 采集业务]         [indoor_nav_ws 的 V0 导航业务]
      XVF3800 / IM72D128                   mission_v0.yaml
      vibration_node                       relative_mission_manager
      ESC/IMU/RC/Battery                   v0_nav_health/watchdog
      auto_bag_recorder                    /v0_nav/* logs
      rosbag                                 |
        ^                                    |
        +------------ 读导航事件 ------------+
```

**本图为逻辑连接**，GPS 注入本身到底在何种物理链路（独立 GPS 串口或 MAVLink）上，须实物核实；不能推断它必然经过 MAVROS。

### 3.1 进程所有权

| 层 | 唯一所有者 | 不允许的行为 |
|---|---|---|
| 飞控串口 `/dev/ttyACM*`/稳定 by-id | 现有 MAVROS（按真实设备选） | V0 再开串口、pymavlink 并联 |
| ROS Master | 一次运行中的 roscore/roslaunch | 各业务各开不相通的 master |
| 自动 rosbag | 现有 `auto_bag_recorder.py` | V0 直接启动/kill rosbag |
| `/mavros/setpoint_position/local` | 一个 V0 航点发布者 | safety 和 mission 同时写、两个任务同时运行 |
| UWB/Fake GPS 注入 | 现有确定的定位发送方 | V0 又创建第二个定位注入方 |
| 麦克风音频和振动 | 原采集进程 | 导航进程占用 USB 音频设备 |

共享 ROS Master/MAVROS 不代表硬件资源完全隔离。CPU、USB、SD 卡/硬盘、文件系统和 ROS 通信都可能互相影响。设计目标是**业务独立与可观测的资源互不干扰**，需要满负载验收。

### 3.2 两个工作空间的启动关系

原则上不需要把现有声音包搬到 `indoor_nav_ws`。两工作空间分别 source（或建立正确的 catkin overlay），并确保它们使用相同 `ROS_MASTER_URI`、可相互访问的 `ROS_IP/ROS_HOSTNAME`、同一 `/mavros` 全局命名空间。SITL `/use_sim_time=true` 不能被带入实机；真机显式 `/use_sim_time=false`。

阶段一**不重构已验证的采集 launch**：由真机最新 `all_sensors.launch` 启动 MAVROS+采集+自动录包；另起一个仅含导航节点的 `indoor_nav_v0.launch`。如果要单独测试导航，则通过**互斥的** MAVROS-only/SITL 基础入口提供 MAVROS，不得与 `all_sensors.launch` 再启动一份。

长期才考虑抽出 `mavros_bridge.launch`；这不是 V0 首轮交付的必要条件。

---

## 4. V0 ROS 包和建议目录

**优先沿用现有 `src/indoor_nav` ROS 包**；命名仅示意，要求 Codex 查阅真实结构后采取最少变更。

```text
~/github/indoor_nav_ws/
├── src/indoor_nav/
│   ├── scripts/
│   │   ├── mission_manager.py              # 历史 ExternalNav：保留
│   │   ├── safety_supervisor.py            # 历史版本：保留
│   │   ├── world_estimator.py              # 历史 ExternalNav：V0 不启动
│   │   ├── relative_mission_manager.py     # V0 新增/或通过现有代码的模式扩展
│   │   └── v0_nav_health.py                # V0 健康观测（可与 manager 合并）
│   ├── launch/
│   │   └── indoor_nav_v0.launch            # 不 include MAVROS
│   └── config/
│       ├── mission_v0.yaml                # 用户唯一主要任务配置
│       └── safety_v0.yaml                 # 可选的基础限制配置
├── tools/
│   ├── sitl_flight_instance.sh             # 历史入口：不覆盖
│   ├── start_v0_fakegps_sitl.sh            # 新增/复用实物协议的集成测试入口
│   └── test_v0_relative_waypoints.py      # 机器可判定结果
├── tests/                                  # 单元/集成测试
├── docs/v0/
│   ├── README.md
│   ├── 00_REPO_AUDIT.md
│   ├── SITL_REPORT.md
│   ├── REAL_AIRCRAFT_SOP.md
│   └── SAFETY_AND_LIMITATIONS.md
└── evidence/v0/                            # 本轮证据与历史证据隔离
```

如果旧项目已经有良好的状态机，可以**少新增文件、多复用逻辑**；不允许为了“清理代码”而改变历史已验收行为。

---

## 5. 相对航点坐标契约（核心）

### 5.1 原点与航向锁定时机

飞手人工起飞并悬停，确认飞控已解锁、定位健康、所需 GUIDED 模式的进入策略已验证后，操作员显式调用 `/v0_nav/start`；程序从 FCU local pose 读取稳定有效的起点 `P0=(x0,y0,z0)` 与四元数 yaw `ψ0`。**任务全程只锁定一次**，不得每经过一个航点重置、不得每次转向随动。

坐标规则：

- YAML `x`：起飞后开始**本次任务时**机头向前（forward），米；
- YAML `y`：该时刻机头向右（right），米；
- YAML `z`：该时刻的 EKF 高度向上（up），米。

任务前方向 `[前, 右]` 与 ROS 机体 `FLU=[前, 左, 上]` 对应关系为 `[x,-y,z]`。

### 5.2 坐标转换公式

在 ROS/MAVROS ENU 中，若 `ψ0` 是 local pose 提取的标准 ENU yaw：

\[
\begin{aligned}
x_d &= x_0 + x_r\cos\psi_0 + y_r\sin\psi_0 \\
y_d &= y_0 + x_r\sin\psi_0 - y_r\cos\psi_0 \\
z_d &= z_0 + z_r
\end{aligned}
\]

等价于 `P_d = P0 + Rz(ψ0) @ [x_forward, -y_right, z_up]`。MAVROS 插件负责把 ROS ENU 指令转换成 MAVLink NED；**不要再在业务层手动交换东/北、正负 Z**。

默认航向目标 `yaw_target=ψ0`，在第一个版本中保持航向。若后续加逐航点 yaw，请明确它相对起点还是绝对 ENU，并增加单元/SITL 测试；不得混合角度单位。

例子：

```
P0 = (2.00, 1.00, 0.85) m
yaw0 = 0°（机头沿 ENU +X）
YAML: x=0.30, y=0.20, z=0.00
ENU target=(2.30,0.80,0.85)m
```

**尤其要注意**：`z=0` 是“保持任务开始时的 EKF Z”，并非离地零米。实际距地高度是否稳定，仍取决于飞控高度源/测距传感器和地面条件。

### 5.3 到达和停留判定

定义 `e_xy = hypot(x−x_d,y−y_d)`、`e_z=abs(z−z_d)`；当 `e_xy < tolerance_xy` 且 `e_z < tolerance_z` 连续满足 `stable_time_s`，记录 `WAYPOINT_REACHED`，随后保持当前目标 `hold_s`，进入下一航点。

若 `max_waypoint_time_s` 到期、定位质量不足、用户切走模式、触发越界/超速，进入锁存中止状态并拒绝自动恢复。任务完成后保持最后安全目标 `FINISH_HOLD`，要求飞手人工退出任务/接管；**即使节点停止发布，飞控也可能保持最后一个位置目标**，不能把杀进程视作刹车。

---

## 6. YAML 配置规范与用户操作

完整可拷贝样例见 `mission_v0_copter.yaml`，主要字段含义：

| 字段 | 含义 | 实现约束 |
|---|---|---|
| `schema_version` | 配置版本 | 版本不兼容则拒绝 |
| `mission_id` | 实验任务唯一标识 | 写日志/事件/rosbag |
| `navigation.source` | `fc_ekf_fake_gps` | 不表示软件自动创建 Fake GPS |
| `reference.axes` | `forward_right_up` | 只在任务开始时锁定 |
| `start_policy.*` | 飞手起飞、手动授权、模式策略 | 自动 ARM/起降必须为 false |
| `control.dry_run_default` | 初次默认不发飞行目标 | 实机显式解除才发 |
| `motion.speed_xy_mps` | 期望慢速运动 | 必须在飞控侧真实生效/可核验 |
| `motion.max_speed_xy_mps` | 飞控限速前置条件 | 必读回/验证，不能只记录 |
| `motion.setpoint_rate_hz` | 连续发布频率 | 建议 5–10 Hz 起测 |
| `arrival.*` | 位置阈值、稳定时长和超时 | 连续到达，不瞬间误触发 |
| `safety.*` | 起点偏移、位姿时间、异常速度 | 不替代物理围栏/FCU failsafe |
| `waypoints[]` | 每一个从相同原点定义的目标 | 全部提前校验，不接受非法位置 |
| `finish.action` | `hold_and_wait_pilot` | 不自动降落、RTL、DISARM |

### 6.1 推荐参数初值（仅作为测试起点）

| 类别 | 初步值 | 注意 |
|---|---:|---|
| 单段位移 | 0.2–0.3 m | 起初用单航点 |
| 期望水平速度 | 0.20 m/s | 必须有可验证的限速执行链 |
| 最大水平速度 | 0.30 m/s | FC 参数/速度目标实际控制 |
| setpoint 发布 | 10 Hz | 与传感器满载测试最大抖动 |
| XY 容差 | 0.12 m | 应结合 UWB/EKF 实测噪声调整 |
| Z 容差 | 0.08 m | 高度传感器稳定性优先 |
| 稳定时间 | 1.5 s | 连续满足误差阈值 |
| 单航点超时 | 15 s | 过短可能造成正常任务误报 |
| 最大起点 XY 偏移 | 0.8 m | 必须小于真实安全余量 |
| 最大起点 Z 偏移 | 0.2 m | 室内地面/顶棚留足安全距离 |
| pose stale 初值 | 0.35 s | 按 FCU 发布率/实际延迟标定 |

这些不是已通过的安全阈值。算法应先输出可测统计分布，再由真机台架和飞行场地据实收紧/调整。所有安全约束都应该支持拒绝不合理 YAML。

### 6.2 速度参数兼容性

特别注意 ArduPilot 新旧参数命名差异：旧版本可能是 `WPNAV_SPEED`（通常 cm/s），新文档为 `WP_SPD`（m/s），还有垂直速度、加速度和 `MAV_CMD_DO_CHANGE_SPEED`。**不能只按名称做字符串替换，还要核实单位和目标固件是否应用于 Guided 位置目标。**

V0 优先采用**飞控预先由人工设置参数 + 程序只读核验**，不在启动时自动刷一批 FC 参数。若未验证速度确实受约束，`START` 必须失败而不是按默认高速飞行。SITL 必须以 FC local velocity/真实路径验证运动速度，持续异常超速要中止。

---

## 7. MAVROS 通信接口和节点所有权

### 7.1 订阅、发布、服务

以下是设计契约；具体话题/类型必须由目标 ROS Noetic 版本的 `rostopic list/type/info` 核实。

| 接口 | 类型 | 用途 |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | FC 连接、ARM、模式 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | EKF 位置/yaw |
| `/mavros/local_position/velocity_local`（检查存在性） | `geometry_msgs/TwistStamped` | 速度监控 |
| `/mavros/global_position/raw/fix`（如启用） | `sensor_msgs/NavSatFix` | Fake GPS 状态辅助 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | **唯一** Guided 位置目标出口 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 若配置允许/明确授权，可请求模式，必须回读 |
| `/mavros/param/get` | `mavros_msgs/ParamGet` | 只读核验关键速度参数 |

优先使用 `/mavros/setpoint_position/local` 而不是自己构造 `SET_POSITION_TARGET_LOCAL_NED` MAVLink 帧；若旧仓库已有成熟 raw setpoint 封装，只能在确认无其它发布者且坐标 frame/type_mask 正确时复用。V0 不需要连续发送速度指令来模拟位置闭环；由 ArduCopter Guided 飞控负责从目标点生成运动。

### 7.2 控制授权与接口

建议使用 `std_srvs/Trigger`：

- `/v0_nav/start`：显式开始；检查授权与所有先决条件，先冻结原点，再稳妥进入执行；重复调用应拒绝。
- `/v0_nav/abort`：请求中止；Latch，不得自动恢复。定位健康时可按通过 SITL 的“保持当前位姿”流程；定位坏时不能假装位姿保持可靠。

真机缺省 `control.dry_run_default: true` 或 launch `enable_control:=false`，可验证参数/健康/坐标但**不发送控制命令**；启用控制必须明确操作。不要把服务调用或 ROS Topic “已经发出”误判为飞控已接受，记录真实 mode 与 EKF 反馈。

### 7.3 状态机

```text
BOOT / IDLE
   |
   | 用户明确 start + 配置通过
   v
WAIT_FOR_READY —检查 MAVROS.connected、armed、EKF pose/速度健康、
   |          初始悬停稳定、GUIDED 模式策略、任务软边界及速度设置
   v
CAPTURE_ORIGIN — 只锁定一次 p0 与 yaw0
   v
HOLD_INITIAL — 先用当前位置作为位置目标，检查模式/响应
   v
EXEC_WAYPOINT[i] — 连续发送固定 ENU target
   |           |
   |           +-- 中间任意条件失效 --> ABORT_LATCHED
   v
ARRIVAL_STABLE — 满足 xy/z 阈值连续 stable_time_s
   v
DWELL — 到点保持 hold_s；记入日志
   |
   +-- 下一个航点 --> EXEC_WAYPOINT[i+1]
   |
   +-- 所有航点完成 --> FINISH_HOLD --> 手动接管 STOPPED
```

用户从 RC 切离 GUIDED 必须立即取消任务，**禁止节点主动切回 GUIDED**。任务状态即使保持在 `ABORT_LATCHED`，仍应持续提供状态事件，协助飞手/地面查看原因。若 MAVROS 本身失联，软件可能无法发送任何模式变更，必须依赖飞控自主 failsafe 与飞手安全处置。

---

## 8. 导航健康与安全要求

### 8.1 健康观测

不得只看 ROS topic 是否存在；检查当前消息是否新鲜、stamp 与时钟是否一致、数值是否有限、pos/yaw 是否合理、水平移动与速度是否明显矛盾；应监控 MAVROS `connected`/`armed`/`mode`。若能获取 UWB 原始状态/更直接的 GPS 状态，可以关联它们判断**持续发送最后一个值的定位冻结**。

注意：悬停真实位置接近不变是正常的，不能仅因坐标连续相等便认定 UWB freeze；传感器静止与速度误差的判断要按实际噪声和静止工况验证。旧 ExternalNav 的 `INVALID_VELOCITY_FRAME` 检查不应不经适配地应用于 V0 GPS 定位。

### 8.2 故障处理矩阵

| 故障 | 导航业务立即动作 | 最后安全保障 |
|---|---|---|
| 手动切出 GUIDED | 停止发目标，任务锁存中止，不重进 Guided | RC 直接接管 |
| 到点超时 | 不发新航点，事件上报 | 定位健康时按验证的悬停策略处理 |
| GPS/UWB 掉线或 EKF 异常 | 中止任务，禁止依赖无效位置保持 | 预先测试的 FCU failsafe / RC 应急 |
| Fake GPS 位置突然跳变 | 拒绝继续并记录 | 机上飞控 EKF/RC，必要时终止飞行 |
| GPS 值持续冻结但消息还在发 | 融合速度/IMU/上游状态联合监测，锁存中止 | 不可只看 header 更新率 |
| 控制节点崩溃 | watchdog 在有通信能力时介入，禁止自动续飞 | FCU 不可依赖 companion 保持安全 |
| MAVROS 连接丢失 | 所有 navigation 停发，采集按自身状态处理 | 飞控独立 failsafe、RC |
| 越过软边界、持续超速 | ABORT，记录真实运动 | 实体防护+人工接管 |
| 声学节点故障 | 航点代码不直接因此崩溃/改变 FC 模式 | 若造成 CPU/通讯退化则导航自身 health 中止 |
| ROS Master 故障 | 所有业务失去 ROS；不可声称 watchdog 必然还在 | FC 独立控制与 RC |

**重要区分**：失效后“停发 setpoint”不保证刹车。ArduCopter 对位置目标可能继续执行最后一条；`GUID_TIMEOUT` 对位置目标的行为也不同于速度/加速度目标。应在 SITL 对停止发布、进程异常退出和实际控制行为分别测试。

### 8.3 真实室内飞行边界

软限位是以**任务原点**定义，不自动等于实验室墙壁位置。启动前应使任务区、旋翼安全包络与人的距离均留足余量；不能因为 YAML 写了 `max_offset_xy_m` 就认为可以在任何房间/任意起点安全运行。飞手持有模式开关，可从 Guided 切至已验证的 AltHold/Stabilize；若室内定位失效，不把 RTL 作为默认保护动作。

---

## 9. 与声学采集并行运行和录包

### 9.1 功能隔离

- 声学 ROS 只负责传感器、MAVROS 遥测、ARM/DISARM rosbag；
- V0 ROS 只负责航点、安全健康、事件；
- 两者不相互 `rosnode kill`、不等待彼此话题作为无条件启动必需项；
- 只通过一个 ROS Master 和唯一 MAVROS 交换数据；
- 真机不要在两个不同 workspace 的 launch 中重复 include `mavros/apm.launch`；
- 原有声音/振动话题和录制频率不能因为新增 V0 而改名/降采样。

### 9.2 原有采集文件的最小改动

历史上传的 `all_sensors.launch` 录包话题已覆盖多类传感器。联合测试时建议额外录制：

```
/v0_nav/status
/v0_nav/event
/v0_nav/origin_pose
/v0_nav/target_pose
/v0_nav/heartbeat
```

可根据数据量选择是否记录 heartbeat。只需要**在机载最新采集 launch 的原 `topics` 参数中追加以上话题**，不重写 `auto_bag_recorder.py` 的核心 ARM/DISARM 逻辑，不把 `rosbag -a` 作为默认替代（大量音频消息容易造成额外负载）。导航参数配置 SHA256、事件时刻、FC 参数快照应该单独保存到 evidence，或以小型事件 topic 发布一次。

### 9.3 资源测试

必须用真实采集负载验证：ROS 图结构中仅一份 MAVROS；单个 setpoint 发布者；CPU 内存峰值；Pose age 最大值/P99；setpoint 周期与最大间隔；ESC telemetry 消息数量；XVF raw 预期 16kHz×6 数据块的缺失率；IM72D128 48kHz×6 数据连续性（如实际已接入）；振动 topic 丢包；rosbag 正常收尾并可回放。

允许导航失败不牵连录包，但如果导航失败意味着飞控接管导致 DISARM，`auto_bag_recorder` 按既有约定自动停包，这是正常行为，不属于任务节点直接杀死录制。

---

## 10. SITL 测试策略：F0 → F1 → F2 → F3

> **严禁**在没完成 Fake GPS 等价 F2 的情况下，仅凭 F1 SITL 默认 GPS 成功就宣布“外接 Fake GPS 自主航点已经验证”。

### F0：旧 ExternalNav 回归

- 检查旧入口、旧世界坐标与旧 `mission_manager/safety_supervisor` 是否仍在；
- 跑基本单元测试/smoke 和最小必要闭环回归；
- 独立保留 `run_06` 证据，禁止覆盖历史 EEPROM/SITL 参数；
- 如果历史完整测试成本高，不需重做全部研究性测试，但必须验证核心旧功能未明显退化。

### F1：SITL 内置 GPS + 相对航点功能

目的：隔离外部定位，先证明任务算法、ROS/MAVROS/Guided 控制接口正确。测试：

1. start 前不产生任何运动目标（除经允许的 dry-run）；
2. 手动模拟起飞/悬停后，锁定同一个 P0/yaw0；
3. 前进 0.3m、右移 0.3m、返回原点；
4. 更换初始 yaw（0°/90°/180°），任务 `forward/right` 方向正确；
5. 飞行途中转航向，任务轴保持固定；
6. 一次进圈但不稳定不足 `stable_time_s` 时不能判到达；
7. 实际速度不越过经验证的限制；
8. 任务结束停留，遥控器切模式后不会重新进 Guided；
9. 超时、非法 YAML、任务越界、异常 pose 等正确拒绝或 ABORT。

录制 ROS bag、Gazebo ground truth（若可用）、FCU DataFlash、参数快照、任务 YAML SHA256 和人/机器都可读的 pass/fail 报告。

### F2：Fake GPS 等价注入链路

目的：真机若使用 UWB Fake GPS，SITL 也必须使 EKF 通过**相同类型的定位入口**获得位置，而不是复用 ExternalNav。先核查真机真实路径，例如：

```
A. Nooploop → Fake GPS 程序 → MAVLink GPS_INPUT → FCU
B. Nooploop → 虚拟 GNSS 串口 → FCU GPS backend
C. 其他已验证路线
```

只选择**实际存在**的一条。若 A，应验证 GPS backend 的 MAVLink 配置（固件中可能叫 `GPS_TYPE` 或 `GPS1_TYPE`，MAVLink 类型通常枚举 14），与 GPS `EK3_SRC1_POSXY=3`；如果 B，应模拟真实 GNSS 串口消息与设备参数。不要自动把全部这些方式都实现。不要把老 `world_estimator → /mavros/odometry/out` 混进新链路。

F2 应独立启动 Fake GPS 测试实例，关闭与之冲突的默认 SITL GPS/ExternalNav 输入，并证明飞控实际接受了模拟的 GPS 数据。若参数、序列或 FCU EKF 状态不符合，测试算失败，而不是因为 ROS mission 能发布 setpoint 就算通过。

故障注入：丢包、间歇延迟（如 50/100ms 作为测试水平）、短时跳点、长时冻结、GPS 质量下降、偏置、导航源中断、重连、MAVROS 丢线。**任何一个关键失效导致不受控续飞则禁止实机 R2。**

若用户手头的 Fake GPS 原始代码未在本工作区且无法获取，应在文档明确写 F2=BLOCKED、缺少的具体入口与下一步实机只读检查；完成其它代码和 F1，不允许伪造通过报告。

### F3：SITL+采集并行与故障演练

- 只运行 MAVROS+导航；只运行 MAVROS+传感器；两业务一起运行；
- 检查一份 MAVROS、一份 ROS Master、单一 setpoint publisher；
- 测试 kill 导航任务、kill 某音频节点、stop rosbag 后的分级行为；
- 使用可用的真实音频设备和振动设备；不具备硬件时模拟代表性 CPU/数据吞吐负载并标注限制；
- 记录 setpoint jitter、FC/ROS 连通性、消息时间戳与 bag 保存质量；
- 证据目录独立，例如 `evidence/v0/f0_...` 到 `evidence/v0/f3_...`。

### 10.1 自动化单元与集成测试矩阵

| 用例 | 标准结果 | 层级 |
|---|---|---|
| YAML 漏字段/未知版本/重复 ID | 拒绝 START，说明原因 | Unit |
| YAML 负速度、负 hold、非数、越界 | 拒绝加载/任务启动 | Unit |
| yaw 0/±90/180° 映射 | 前/右/上正确 | Unit |
| 原点冻结与中途转头 | 目标不重算原点 | Unit + F1 |
| 时间戳倒退/超时 | 不使用旧 pose | Unit + F1 |
| 到达瞬间越过容差 | 不误判到达 | Unit + F1 |
| START 重复请求 | 拒绝重入 | Unit |
| ABORT 之后 START | 需人工复位、不能自动继续 | Unit + F1 |
| RC 从 Guided 切走 | 马上终止控制、不重新切回 | F1 + R0 |
| MAVROS/Fake GPS 掉线 | 导航锁存、FCU 按已验证 failsafe | F2 |
| 位置冻结但消息刷新 | 结合额外质量/速度检查，或显式记缺口 | F2 |
| 单航点/方形/返回 | 实际轨迹和容差合格 | F1/F2 |
| 声学进程停止 | 导航业务不直接被杀 | F3 |
| 导航进程停止 | 声学/录包业务不直接被杀 | F3 |
| 双 MAVROS/双目标发布者 | 运行前检测并拒绝有风险控制 | Unit + F3 |
| 满负载 rosbag | 音频连续、事件可关联、导航 jitter 合格 | F3/R4 |

---

## 11. 真机 SOP：只有人执行，Codex 不自动执行

### R0：拆桨台架（最低风险）

确认飞控固件、Fake GPS 物理输入、MAVROS 串口和唯一占用者、ROS 实时/仿真时钟、`/mavros/state`、`/mavros/local_position/pose`、Yaw、健康和遥控器模式开关。手持在安全场地沿机头前、右、上移动，记录 ENU 变化；多次改变 yaw 再验证；测试断 UWB、断 MAVROS、超时等响应。**无桨禁止声称已验证真实动力学或悬停。**

### R1：飞手人工低空悬停

隔离人员、足够空间、低速限制、提前确定安全边界，飞手通过正常流程解锁/起飞，先验证 UWB Fake GPS 下稳定位置和高度；V0 节点仅 `dry_run`，不得下航点。

### R2：单一短航点（首次自主控制）

经过人工授权、实际 GUIDED 模式确认、已核验限速和 failsafe 后，启用 V0 控制，单个 0.2–0.3m 相对目标；飞手始终准备切回已验证的 AltHold/Stabilize。记录实际轨迹、速度、最大偏差及事件。失败不继续扩大路线。

### R3：小范围往返/方形

短时间小正方形、重复几次；核查起点固定、到达时间、静止期间位置/声学/ESC 数据一致性。

### R4：全部传感器联合运行

按机载最新 `all_sensors.launch` 运行 XVF3800、IM72D128（如已部署）、RS485、ESC 与 rosbag，然后启动 V0 导航，执行 R2/R3 已验收轨迹。确认录包数据完整、导航延迟/抖动与单独运行差异可解释。

### 11.1 真机放行闸门

- F0/F1 完成（旧代码未破坏，基本控制功能正确）；
- F2 完成（SITL 与真机的 Fake GPS 输入接口等价且 EKF 确认）；
- F3 至少完成有代表性的并行验证（真机 R4 再完成真实硬件满负载）；
- R0 拆桨坐标方向、failsafe、RC 接管和参数检查全部通过；
- R1 飞手先确认手动/定点悬停；
- 只允许有安全试飞人员、飞行许可/物理隔离且准备充分时进入 R2。

---

## 12. 运行与检查命令草案

> 下列 `indoor_nav_v0.launch`、`mission_v0.yaml` 是**未来 Codex 要创建的文件名**，不是现在已经存在的命令；从本机实际 ROS 环境和包名核查后执行。

### 12.1 Git 建分支

```bash
cd ~/github/indoor_nav_ws
git status --short
git branch --show-current
git log --oneline -5
# 工作树无未保存修改、基线确认后：
git switch -c feat/arducopter-v0-relative-waypoints
```

### 12.2 测试环境信息收集

```bash
source /opt/ros/noetic/setup.bash
rospack find mavros
rosnode list
rostopic list | grep '^/mavros/' | head -60
rostopic type /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
rostopic echo -n 1 /mavros/state
rostopic info /mavros/setpoint_position/local
```

### 12.3 真机双业务分开启动（导航初始只读）

终端 A 启动**机载已验证的最新采集系统**（具体 workspace source、package 和 launch 的实际路径需要现场确认）：

```bash
# 本终端 source 现有声音 ROS 工作区
roslaunch rs485_driver_sound all_sensors.launch
```

终端 B 启动新导航业务，共用 ROS Master 和 MAVROS：

```bash
source /opt/ros/noetic/setup.bash
source ~/github/indoor_nav_ws/devel/setup.bash
roslaunch indoor_nav indoor_nav_v0.launch enable_control:=false
```

查看：

```bash
rosnode list | grep -E 'mavros|v0_nav|auto_bag'
rostopic info /mavros/setpoint_position/local
rostopic echo /v0_nav/status
```

正确状态：**只有一份 MAVROS**；真机只读模式绝不下 Guided 运动目标。正式实飞时如何切控制开启，必须按 Codex 新文档/用户现场授权执行。

### 12.4 任务服务接口（实现后）

```bash
rosservice call /v0_nav/start '{}'
rosservice call /v0_nav/abort '{}'
```

只有在模式、飞手确认、定位、速度和安全门槛全部满足时才能 start。这里**不包含 ARM/Takeoff/Land 指令**。

### 12.5 日志分析

从 rosbag 里至少联合分析：

```
/mavros/state
/mavros/local_position/pose
/mavros/local_position/velocity     # 真实 topic 以本机为准
/mavros/esc_telemetry
/sound_sensor/audio_raw
/motor_mic/audio_raw               # 如实机已部署
/vibration_data
/v0_nav/event
/v0_nav/target_pose
/v0_nav/origin_pose
```

统计每段航点的 RMS 位置误差、最大偏差、到达时间/悬停时长、真实速度和 ESC RPM；以 `mission_id`、YAML SHA256、bag 路径、FC 参数快照作为实验版本键。不同 UWB 重启的任务原点可能不同，**V0 保证相对于本次任务起点复现，不能保证每次都抵达实验室相同物理绝对点**。

---

## 13. 工作成果及验收定义

### 13.1 Codex 本地必须交付

1. 新 Git 分支及真实状态报告，旧成果不被覆盖；
2. V0 节点、YAML、独立 `launch`、README、可观测状态/事件/目标；
3. 仅通过 MAVROS 的 Guided 相对航点功能，单一控制发布者；
4. YAML 校验、坐标转换单元测试、任务状态机/健康/超时/安全测试；
5. 旧 ExternalNav F0 与新 GPS F1 的真实测试；如果 Fake GPS 接口已可得，F2 等价集成；
6. 与采集并行的 F3 测试和资源/时间对齐证据；
7. 可让人逐步操作的真机 SOP（绝不替用户自动真机上电试飞）；
8. 所有通过/失败/阻碍的证据路径；任何未通过不伪造。

### 13.2 V0 成功的定义

**在不破坏原声学采集功能的前提下，机载 YAML 控制 ArduCopter 利用健康的 UWB Fake GPS/EKF local pose，以低速执行相对于一次锁定任务起点和初始机头方向的小航点序列；全程 MAVROS 只有一个实例，声学采集和导航彼此独立，任务状态与声学/ESC/振动能够在实验数据中对齐，并且 SITL 的关键故障和人工接管得到验证。**

### 13.3 V0 不保证

不保证全球绝对位置，不保证跨天同一物理地点重复，不保证 Fake GPS 的厘米级精度，不保证软件围栏就是硬件围栏，不保证控制节点断链时可以主动控制飞行器，不保证无人工接管时室内安全。

---

## 14. 参考资料与来源边界

### 来自本项目的材料（项目事实和历史记录）

- `all_sensors.launch`、`auto_bag_recorder.py`、`xvf3800_node.py`、`vibration_node.py`（本轮附件；采集软件的可核查快照）。
- `UAV_acoustic_hardware_baseline(1).md`（声音硬件、ROS topic、两套麦克风并行设计）。
- `室内UWB定位与航点飞行_V1方案.md`（原 ExternalNav/V1 设计及测试路线）。
- `indoor_nav_ws_V0_ArduCopter_分支与SITL实机实施方案.md`、`无人机V0相对航点_双ROS并行架构设计.md`（先前规划；具体文件与版本应再次本地核对）。
- 先前 `indoor_nav_ws` SITL 聊天记录中的 run_06、streamrate 和 velocity frame 经验（**历史报告，需在当前本机追踪相应证据**）。

### 经官方资料支撑的接口知识（需要按目标固件复核）

- Copter Guided 命令与 `SET_POSITION_TARGET_LOCAL_NED`：https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- Copter Guided 模式和速度行为：https://ardupilot.org/copter/docs/ac2_guidedmode.html
- ArduPilot EKF Source Selection：https://ardupilot.org/copter/docs/common-ekf-sources.html
- 非 GPS/ExternalNav 与 `GPS_INPUT` 建议：https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
- 新旧参数名/单位的变更：https://docs.ardupilot.org/copter/docs/common-param-name-changes.html
- MAVProxy GPS_INPUT 注入示例（仅当真实 Fake GPS 使用该协议时适用）：https://docs.ardupilot.org/mavproxy/docs/modules/GPSInput.html

### 当前尚未验证的实现细节（不得擅自认定）

- 用户真机的 Fake GPS 实际协议/串口路由与上游节点；
- 当前 ArduCopter 固件版本和所有 EKF/GPS/Guide 速度参数；
- 当前机载最新版 `all_sensors.launch` 是否已加入 IM72D128；
- V0 新分支中的具体文件名、节点功能、实测 SITL 报告；
- 真机安全空间、飞手接管开关及真实限速所需裕度。

---

**一句话工程约束：只增量改 `indoor_nav_ws`，只用共享 MAVROS 发 ArduCopter Guided，相对航点靠 YAML；不碰已有声学采集和 rosbag 的核心行为；SITL 先验证真正的 Fake GPS 链路，再由飞手逐级上真机。**
