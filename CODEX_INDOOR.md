# indoor_nav 室内 UWB 定位与自定义航点：Codex 集中修复任务书

> **交付对象**：本地 Codex（有权访问本机仓库）  
> **仓库**：`~/github/indoor_nav_ws`  
> **环境**：Ubuntu 20.04 / ROS Noetic / Gazebo Classic 11 / ArduPilot Copter SITL 4.5.7 / MAVROS / QGroundControl  
> **当前任务**：集中排查并修复 SITL 无 GPS 外部定位、坐标系、安全围栏、航点任务启动及测试流程。  
> **执行原则**：直接阅读仓库、修改源码、补齐自动化测试、运行无解锁验证；**不要自行解锁、起飞、取消飞控安全检查或修改真实硬件参数**。所有 ARM/TAKEOFF 操作需操作员明确授权。  
> **资料优先级**：本地仓库当前文件 > 本任务书中引用的历史源码快照 > 聊天日志与推测。

## 0. 给 Codex 的直接指令

请作为该项目的开发者，**在本地仓库直接完成修复和测试，而不是只输出建议**。先读取 `git status`、当前源码、README、launch、YAML、SITL 启动脚本及测试，确认实际版本；不要覆盖用户尚未提交的修改。建立可复现的诊断脚本，集中解决下列问题，并提供最终可运行命令、测试结果、剩余风险。遇到不确定之处先用代码和 ROS 运行时证据定位；禁止通过盲目放宽安全围栏、强制 `READY`、屏蔽 EKF/ARM 检查来使流程表面通过。

### 必须完成的目标

1. 在 Gazebo SITL 中实现 **GPS_TYPE=0 的 ExternalNav 冷启动**，明确 EKF Origin 建立方式，确认飞控持续融合外部位置。
2. 确认 Mock UWB **独立读取 Gazebo 模型真实位姿**，且运动时输出正确变化，不能从 `/mavros/local_position/pose` 反向生成 Mock UWB（避免正反馈自证）。
3. 修复 `safety_status: WAIT / XY_OUTSIDE_FENCE` 的实际根因，明确加载的围栏配置、当前 world 坐标、边界以及为何越界；不得简单扩大边界或关闭检查。
4. 确保 `world_lab` → `odom` → MAVROS → ArduPilot EKF 的变换方向、姿态和高度一致，排查约 0.2 m 的 EKF 与 ROS 位置差异；**以动态增量、坐标轴方向和观测日志验证，而不是强制令两套原点数值相等**。
5. 保留 `/indoor_nav/safety/acknowledge`、`/indoor_nav/mission/start`、`/indoor_nav/mission/stop` 的安全语义，形成完整、明确、可重复的 **SITL 启动 → 检查 → 人工授权起飞 → GUIDED → 航点 → 停止/降落** 流程。
6. 添加无需 ARM 的自动化测试，覆盖围栏边界、初始高度、坐标变换、定位静止/移动、故障注入、任务服务状态机；输出一份操作说明和验收报告。

## 1. 项目目标与系统架构

项目目标：在约 3×3 m 室内实验空间中，使用固定 `world_lab` 世界坐标系重复执行航点。真实硬件使用 Nooploop LinkTrack UWB（飞机 Tag T0、固定参考 Tag T1、多个 Anchor）获得 XY，MicoAir MTF-01P 的下视测距/光流接飞控，姿态/航向由飞控提供。机载 ROS 管理定位、外部里程计、任务与安全监督。

当前仿真链路：

```text
Gazebo /gazebo/model_states（真实模型位姿）
        ↓
/mock_uwb_node ───→ /nlink_linktrack_nodeframe2、/nlink_linktrack_anchorframe0
        └─────────→ /indoor_nav/sitl/mock_range (sensor_msgs/Range)
                            ↓
/world_estimator ← 模拟 UWB、Range、/mavros/imu/data
        ├── /indoor_nav/world_pose (world_lab)
        ├── /indoor_nav/localization_status
        ├── /indoor_nav/nav_from_world、/indoor_nav/z_offset_m
        └── /mavros/odometry/out（30 Hz）→ MAVROS → MAVLink ODOMETRY → EKF3

/mission_manager ← mission.yaml、定位、飞控状态、安全状态
        └── /indoor_nav/desired_setpoint
                    ↓
/safety_supervisor ← world_pose、围栏、定位、飞控状态、人工确认
        └── /mavros/setpoint_position/local（仅条件允许时转发）
```

重要：`/indoor_nav/sitl/mock_range` 是 **模拟测距高度**，不是 UWB XY；Mock UWB 的 XY 由模拟 LinkTrack 话题承载。不能因 Range 只有一个距离值就误判整个 Mock UWB 没有位置输出。

## 2. 当前已验证的运行结果（2026-10-09）

### 2.1 ArduPilot / MAVROS 参数

```text
ArduCopter V4.5.7, gazebo-iris, --model gazebo
GPS_TYPE       = 0        # 已关闭 GPS 后端
VISO_TYPE      = 3
VISO_POS_X/Y/Z = 0/0/0
EK3_SRC1_POSXY = 6        # ExternalNav
EK3_SRC1_VELXY = 0        # 无外部速度
EK3_SRC1_POSZ  = 1        # Baro
EK3_SRC1_YAW   = 1        # Compass
AHRS_EKF_TYPE  = 3
EK3_ENABLE     = 1
```

关闭 GPS 之前，MAVProxy 曾明确输出：

```text
AP: AHRS: EKF3 active
AP: EKF3 IMU0 origin set
AP: EKF3 IMU1 origin set
AP: VisOdom: yaw shifted 359 to 1 deg
AP: EKF3 IMU0 is using external nav data
AP: EKF3 IMU1 is using external nav data
```

**注意**：这些是先前启动的日志，不能冒充 GPS 关闭后那次启动的证据。需要在 `GPS_TYPE=0` 的完整冷启动日志中重新核验 ExternalNav 使用和 EKF 创新量。

### 2.2 GPS 禁用后 ROS 观测

```text
/indoor_nav/localization_status:
  calibrated: true
  transform_locked: true
  healthy: true
  armed: false
  flight_controller_connected: true
  uwb_age_s: ~0.026
  laser_age_s: ~0.035
  eop_xy_m: ~0.01414
  reason: OK

/mavros/odometry/out: ~30.000 Hz
/mavros/global_position/gp_origin:
  lat: -35.3632621
  lon: 149.1652374
  alt: 603.4386373  # SITL 参考原点，不是真实地理位置

/indoor_nav/world_pose (frame_id=world_lab):
  xyz ≈ (0.00144, 0.00013, -0.00012) m
/mavros/odometry/out xyz ≈ (-0.00049, 0.00134, -0.00007) m
/mavros/local_position/pose xyz ≈ (0.1905, -0.1990, -0.0664) m
```

EKF 本地 XY 曾从约 `(0.213,-0.233)` 缓慢变化到 `(0.1905,-0.1990)`，而 ROS Odometry 接近零。此差异可能是局部原点、航向对齐、EKF 初始化/收敛、观测未持续融合等原因，**不要直接认定为传感器安装偏移**。要求记录时间同步的数据并做动态位移测试。

### 2.3 MAVROS Odometry 参数

```yaml
/mavros/odometry:
  estimator_type: 3
  frame_tf:
    desired_frame: ned
```

历史消息头为 `frame_id: odom`、`child_frame_id: base_link`。检查静态 TF 发布方向和 MAVROS odometry 插件是否按预期做 ENU/FLU ↔ NED/FRD 变换。历史 launch 包含 `odom_ned`/`odom` 和 `base_link_frd`/`base_link` 静态变换，但以当前仓库与运行时 `tf2` 为准。

### 2.4 当前阻塞：安全监督拒绝航点

```text
首次：
  state: WAIT
  ready: false
  abort_latched: false
  reason: OPERATOR_NOT_ACKNOWLEDGED

rosservice call /indoor_nav/safety/acknowledge:
  success: true
  message: Operator acknowledged. This does NOT arm or start flight.

随后：
  state: WAIT
  ready: false
  abort_latched: false
  reason: XY_OUTSIDE_FENCE

rosservice call /indoor_nav/mission/start:
  success: false
  message: Safety supervisor not READY
```

**优先排查这个矛盾**：历史 `safety_supervisor.py` 的围栏判断使用 `/indoor_nav/world_pose`，当时该位置接近 `(0,0)`；历史 SITL 专用 `safety_sitl.yaml` 的 XY 为 `[-1.10,1.10]`，理论上应该通过。历史实机 `safety.yaml` 的 XY 为 `[0.40,2.60]`，原点附近会触发 `XY_OUTSIDE_FENCE`。**这高度提示加载了错误的安全配置或参数被覆盖**，但尚未验证，不能直接宣称根因已确定。

历史源码快照还显示：`indoor_nav_sitl.launch` 将 `safety_file` 设为 `config/safety_sitl.yaml`，通过 `indoor_nav.launch` 加载至 `/indoor_nav/safety`；因此必须检查当前实际 `rosparam get /indoor_nav/safety`、launch 参数覆盖、重复启动节点、ROS master 中残留参数、是否使用错误 launch、是否重启过节点，以及 `self.box` 的实时值。

## 3. Codex 的实施任务

### P0：静态检查与复现（先做）

1. 执行 `git status --short`、`git log -1 --oneline`，记录工作树和分支；不得覆盖未提交修改。
2. 查阅当前 `src/indoor_nav/scripts/{world_estimator.py,mock_uwb_node.py,safety_supervisor.py,mission_manager.py}`，`src/indoor_nav/src/indoor_nav/core.py`，`src/indoor_nav/config/{safety.yaml,safety_sitl.yaml,mission.yaml,localization.yaml,sitl.yaml,ardupilot_expected.yaml}`，`src/indoor_nav/launch/*.launch`，`tools/*.sh`、`tools/sitl.env`、`tools/set_ekf_origin_once.py`。
3. 查找 `XY_OUTSIDE_FENCE` 的全部分支、`inside_box` 的行为、参数命名空间与覆盖顺序；从日志打印 **position、fence bounds、frame_id、armed/mode、配置来源**，避免只给一个字符串错误。
4. 验证 Mock UWB 真实订阅 `/gazebo/model_states` 且 `model_name=iris`，确认其发布 XY 由 Gazebo 真值生成；避免误以为 Range 话题是全部 UWB 数据。
5. 找出 `world_lab`、`odom`、`map`、`odom_ned`、`base_link`、`base_link_frd` 的定义及所有转换，绘制坐标链；检测 TF 环、重复发布和父子反向。
6. 明确启动工具是否有 `--wipe`、是否重置 `GPS_TYPE`、是否加载旧参数、是否重复运行 `world_estimator` 或 `safety_supervisor`。

### P1：修复安全围栏和状态诊断

1. 将 SITL 与真实飞行的安全配置明确隔离，避免 SITL 意外加载实机围栏 `[0.40,2.60]`；不要直接修改实机安全边界。
2. 在启动时输出最终生效的 YAML 文件和 geofence 数值；运行时可查看已加载参数，并检测配置与任务坐标系是否一致。
3. 使安全状态包含可解释诊断：当前 world XYZ、围栏 min/max、检查失败维度、配置 profile、飞控 armed/mode、定位有效期。可扩展独立诊断话题/日志，尽量保持原消息兼容。
4. 保留 **地面未解锁时低于 `z_min` 可以等待起飞** 的原设计，但绝不能因此放过 XY 越界；已解锁/执行航点时必须严格执行 Z、XY 与预测围栏。
5. 对 `acknowledge`、`reset`、`abort_latched`、`mission/start` 的状态依赖编写测试。`acknowledge` 成功并不意味着 `READY`；应明确报告下一项未通过的检查。
6. 检查默认 `mission.yaml` 中 `(1.0,1.0,0.8)` 等航点与 SITL 围栏和 3×3m 环境边界/安全裕量的关系，防止任务目标落在可飞空间之外。

### P2：定位链路与冷启动

1. `GPS_TYPE=0` 的全新 SITL 进程下验证 EKF Origin 来源（人工工具、SITL 默认、其他 MAVLink 消息），明确与真 GPS 无关；不得把 GPS 关闭前日志当成无 GPS 的证据。
2. 验证 `world_estimator` 在 FC 本地位置尚未出现时也能独立完成校准、输出 ExternalNav；消除冷启动环形依赖。
3. 记录 `/gazebo/model_states` 真值、Mock LinkTrack、`world_pose`、`odometry/out`、`local_position/pose` 的时间同步位置序列。
4. **未解锁动态验证**：在可控的 Gazebo 模型位姿输入或 mock 真值测试桩中，施加缓慢平移，比较 XY 方向、幅值、时间延迟、坐标变换、EKF 响应。瞬移只可用于链路验证，不可直接用来评价实际控制精度。
5. 校验 Odometry 的协方差语义：XY 有可信方差，未观测的速度、Z、yaw 不应以虚假的高置信度提供给 EKF；验证插件是否需要有效 TF 和时间戳，查看 MAVROS/飞控错误日志。
6. 如果 DataFlash 不生成，检查 `LOG_BACKEND_TYPE`、`LOG_DISARMED` 和 SITL 工作目录；在未解锁状态收集 ExternalNav 接收/融合、EKF 创新或等效证据，记录具体固件支持的日志消息类型。

### P3：航点启动与停止流程

1. 保持原始设计：`/indoor_nav/mission/start` **只启动任务，不自动 ARM、不自动 TAKEOFF、不强制改模式**；`/mission/stop` 停止目标输出，不能声称已降落。
2. 给用户提供 `tools/indoor_nav_preflight.sh` 或 Python 等价工具，自动只读检查 ROS 节点、服务、定位健康、GPS/EKF/VISO 参数、origin、TF、围栏 profile、任务目标合法性、Mock UWB 真值依赖、MAVROS 连接状态；输出 `PASS/WARN/FAIL` 和修复提示。
3. 提供独立的 `tools/indoor_nav_status.sh`，一次显示关键状态和拒绝启动原因，避免用户逐条运行十几条命令。
4. 提供 `tools/indoor_nav_sitl_smoke_test.sh`，只执行 **DISARMED** 的端到端定位、服务拒绝路径和静态/动态数据测试；默认不发送解锁、起飞、位置 setpoint。
5. 如需新增辅助启动脚本，默认只启动/检查 ROS+SITL；涉及 ARM/TAKEOFF 的功能必须独立命令、清晰确认、显式 SITL 环境验证和人工授权，不能隐藏在 `/mission/start` 或开机脚本中。
6. 文档说明：任务在 `world_lab` 中是绝对航点；当前初始世界位置可能在 `(0,0)`，但任务首航点是 `(0.5,0.5,0.8)`；必须验证起飞点、航点、围栏、姿态和高度参考一致。

### P4：自动化测试和验收

至少覆盖以下测试：

| 测试 | 预期结果 |
|---|---|
| 未确认操作员 | `WAIT / OPERATOR_NOT_ACKNOWLEDGED`，禁止任务 |
| 确认后、位置在 SITL XY 围栏内、未解锁且低于 `z_min` | 可满足围栏预起飞条件；其余条件仍需逐项通过 |
| 加载实机围栏而位置在 `(0,0)` | 明确报告 profile/边界不匹配，禁止任务 |
| XY 边界外 | 拒绝任务，不修改/扩大边界 |
| 外部定位丢失、延迟、跳点、冻结 | 触发相应健康或安全机制，不持续输出不安全 setpoint |
| 未解锁调用 `/mission/start` | 明确拒绝，不能 ARM |
| 非 GUIDED 调用 `/mission/start` | 明确拒绝，不能切模式 |
| Mock UWB 输入位置变化 | `world_pose` 与 Odometry 的相应位置增量符合变换 |
| Odometry 坐标系转换 | ENU/NED、FLU/FRD 方向正确，静态 TF 无重复/反向 |
| 关闭 GPS 冷启动 | ExternalNav、origin、本地位置可用，且有飞控融合证据 |
| 安全状态异常时 stop/abort | 不继续发送任务 setpoint；记录模式切换/人工接管要求 |

如需要集成 Gazebo 模型位姿测试，请使用隔离测试世界、测试桩或明确标注为仅仿真的脚本，确保不会作用于真实飞控。

## 4. 现有运行入口和可复用命令

```bash
cd ~/github/indoor_nav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# SITL（启动前先核实工具脚本当前实现）
./tools/start_indoor_sitl.sh
./tools/stop_indoor_sitl.sh

# 定位与安全状态
rostopic echo -n1 /indoor_nav/localization_status
rostopic echo -n1 /indoor_nav/safety_status
rostopic echo -n1 /indoor_nav/world_pose
rosparam get /indoor_nav/safety
rosnode info /mock_uwb_node
rosnode info /safety_supervisor

# 只在未解锁并人工确认安全配置时执行
rosservice call /indoor_nav/safety/acknowledge

# 航点服务（仅已起飞、GUIDED、全部检查通过时才允许执行）
rosservice call /indoor_nav/mission/start
rosservice call /indoor_nav/mission/stop
```

历史 `tools/sitl.env` 具有 `EXTERNAL_NAV_ENABLED="true"`、`SITL_MODEL="gazebo"`、`GAZEBO_MODEL_NAME="iris"`、`MAVROS_UDP_PORT="14552"`。实际值以本地仓库为准。

## 5. 必须遵守的约束

- 不将 `XY_OUTSIDE_FENCE` 通过关闭 geofence、扩大飞行空间、硬编码 `READY` 或篡改定位值来“修复”。
- 不将模拟 UWB 从飞控 EKF 输出生成，避免数据闭环自证。
- 不随意设置 `VISO_POS_X/Y/Z` 抵消世界坐标偏差；这是机体传感器安装偏移，不是坐标原点补丁。
- 不把 `EK3_SRC1_POSXY=6` 或有 `/mavros/local_position/pose` 单独当成“融合成功”的充分证据。
- 不把 SITL 安全设置复制到真实飞行配置；真实硬件需要重新标定、检查光流/测距和参数。
- 不自动 ARM/TAKEOFF，不绕过预解锁检查；禁止在真实机上自动测试。
- 不改动用户无关项目、不清理未提交文件、不执行破坏性 `git reset --hard`。
- 若无法运行本机 ROS/SITL，仍完成静态修复与单元测试，并明确说明未完成的动态验收项，不得虚报通过。

## 6. 最终交付格式

请在仓库中直接提交工作树修改（是否 Git commit 由用户决定），至少交付：

1. 修复后的源码/launch/YAML，保留原接口或记录兼容性变更。
2. `docs/SITL_DEBUG_AND_FLIGHT.md`：一页式启动、无 GPS 校准、检查、人工起飞、GUIDED、启动/停止航点、紧急处理命令。
3. `tools/indoor_nav_preflight.sh`、`tools/indoor_nav_status.sh`（或功能等价脚本），无需 ARM。
4. 新增/更新测试，提供精确执行命令与 PASS/FAIL 结果。
5. `docs/ACCEPTANCE_REPORT.md`：问题根因、改动文件、运行日志、静态与动态测试结果、未解决问题及安全限制。
6. 最后在 Codex 回复中用简短中文总结：**为什么出现 XY_OUTSIDE_FENCE、改了什么、怎么启动、如何验证、目前能否安全进行 SITL 航点飞行**。

## 7. 优先级与执行顺序

**先解决能确定的阻塞，再推进动态飞行：**

```text
检查工作树与源码
  → 复现/定位 safety.yaml 与 safety_sitl.yaml 实际加载情况
  → 修复 XY_OUTSIDE_FENCE 的配置/坐标根因
  → 加入一键状态与预检工具
  → 验证 Mock UWB 跟随 Gazebo 真值
  → 验证 ExternalNav → EKF3 动态响应和无 GPS 冷启动
  → 自动化 DISARMED 测试
  → 人工审阅结果，决定是否授权 SITL 低空飞行
  → 再验证 GUIDED 自定义航点
```

**请现在开始执行，不要停留在分析建议。** 若发现本任务书与当前代码不一致，以实际源码为准并在验收报告中说明差异。
