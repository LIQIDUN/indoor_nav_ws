# 给 Codex 的完整开发任务：indoor_nav_ws / ArduCopter V0 相对航点

> **把本文完整粘贴到本地 Codex 窗口**。这是实施任务，不是讨论提纲。先审计本地真实代码，再在新分支实施、测试和交付。本文要求中的具体文件名若与仓库实际不一致，以本地实际结构为准，但不能擅自改变架构约束。
>
> 配套规范：`ArduCopter_V0相对航点_完整项目设计与验收文档.md`；样例：`mission_v0_copter.yaml`。请先阅读配套规范，然后实施。

## 0. 你的角色与工作方式

你是一名精通 ArduPilot ArduCopter、MAVLink、ROS1 Noetic、MAVROS、Gazebo/SITL、Python3、catkin、Git 和实机安全的机器人系统工程师。用户已经授权你在**本地** `~/github/indoor_nav_ws` 上建立新分支、修改代码、编写测试并运行适用的本地 SITL；**未授权**你擅自更改真机飞控参数、自动实机解锁/起飞、覆盖采集工作区、删除历史证据、向远端推送分支或直接开展实机飞行。

这是一个代码交付任务。不要只输出架构思路；要尽可能在当前本地环境完整开发、运行测试、整理文档和提交阶段结果。遇到没有实机硬件、真实 UWB Fake GPS 实现暂时缺失等阻碍时，完成所有不依赖该条件的工作，提供可验证测试、明确标注阻碍，不得伪称已通过 Fake GPS 集成或真机测试。不要对用户提与已有项目资料已经回答的问题；发现重要现实差异时把检查事实写进日志。

**首要原则：最小侵入，保护已验证成果和录包。**

## 1. 目标和非目标

目标：在既有 `indoor_nav_ws` 代码基础上新增一个 V0 航点运行模式，让 **ArduCopter** 使用既有 **UWB → Fake GPS → 飞控 EKF3** 获得局部位置，由机载计算机通过**现有 MAVROS**读取位姿和执行 Guided 相对航点；所有航点和限制从机载 YAML 读取；可独立启动/停止；与既有 ROS 声学/振动/ESC 采集与 ARM/DISARM 自动 rosbag **同时运行、业务解耦**。

V0 定义：飞手人工 ARM/起飞到稳定悬停，明确授权开始任务；冻结任务起点位置和初始机头方向；按 YAML 的前/右/上相对**任务起点**坐标，依次执行短距离航点、到达稳定判定和悬停停留；任务结束保持最后安全目标，等待遥控器接管。默认**不自动 ARM、起飞、降落，不依赖固定实验室 `world_lab`/Ground Tag 校准，不重写 UWB 定位、不做电机底层控制**。

非目标：不引入 pymavlink、DroneKit、第二个 MAVROS、第二个 ROS Master（同一次部署）、第二个 FCU USB 串口占有者；不把外部视觉降落项目 `vtol_vision_bridge` 搬过来；不改 XVF3800 或 IM72D128 采样率/位宽/话题；不重构旧 ExternalNav 为 Fake GPS；不新增过度复杂的自主导航框架。

## 2. 第一步：本地仓库审计与安全分支

在动文件之前执行并保存结果：

```bash
cd ~/github/indoor_nav_ws
pwd
git status --short
git branch --show-current
git log --oneline -10
find . -maxdepth 3 -type f | sort | sed -n '1,220p'
```

1. 确认这是预期 catkin 工作区/仓库，核查 `src/indoor_nav`、`mission_manager.py`、`safety_supervisor.py`、`world_estimator.py`、所有 `launch`、`config`、`tools`、SITL 参数、测试工具和 `evidence/`。如果名称不同，用真实名称并在文档解释。
2. 查清分支基线：优先以原有成功的 ExternalNav SITL 代码版本为基线，确认此前 `run_06` 证据是否存在以及代码提交是否可追溯。此前文档记载 `run_06`（2026-10-09）为 10/10 航点、约 53.13 s、ExternalNav；**这只是项目历史记录，不可冒充本轮验证**。
3. **若工作树有未提交修改**，先列出文件与影响，绝不 `git reset --hard`、`git clean -fd` 或覆盖；使用不会破坏用户修改的方法保护状态再建分支。若环境许可且工作树干净：

```bash
git switch -c feat/arducopter-v0-relative-waypoints
```

4. 不覆盖原 ExternalNav 文件、SITL EEPROM、历史日志、模型和证据；新增 V0 独立 YAML / launch / SITL 参数 / evidence。只在必要且有回归测试时抽取旧代码共享逻辑。
5. 在 `docs/v0/00_REPO_AUDIT.md` 写真实目录结构、ROS/MAVROS/ArduCopter 版本、SITL 入口、Fake GPS 的已知来源、可以直接复用的函数、冲突点和新分支提交。

## 3. 第二步：查清现有导航与 Fake GPS 实际接口

审计：

- 旧 `mission_manager` 怎么启动、自动 ARM/起飞、切 Guided、发布 setpoint、计算到达、超时/中止；哪些代码能在不影响 ExternalNav 基线的前提下复用。
- 旧 `safety_supervisor` 是否强制检查 `world_lab`、ExternalNav 健康、`/indoor_nav/calibrate`、EKF velocity frame、UWB freeze。V0 不得启用不适用的旧安全先决条件。此前历史中曾有 `INVALID_VELOCITY_FRAME` 和静止时速度不一致误报，防止照搬阈值产生错误中止。
- 真机 UWB Fake GPS 究竟由谁生成/输出，经过**飞控 GPS 串口、MAVLink `GPS_INPUT`、`HIL_GPS`、MAVROS、还是其他协议**，与 Pixhawk 和 MAVROS 各自占用哪个物理端口。**没有代码或设备证据绝不猜测协议。** V0 导航只使用飞控**已融合**的 local pose，不再注入一个新的 GPS/ExternalNav 数据源。
- 原有 SITL 接口和参数；旧 ExternalNav SITL 已有命令（若在真实仓库存在）可能是：

```bash
SITL_FLIGHT_COVARIANCE_ADAPTER=true ./tools/sitl_flight_instance.sh
```

这用于**旧 ExternalNav 回归**，不是新的 Fake GPS 验收命令。确认现存 SITL ROS Master、Gazebo、MAVProxy 端口，避免已有 `start_indoor_sitl.sh` 与隔离实例冲突。已记录过 11311/11321 ROS Master 与 SITL5760/Gazebo9002/9003 的互斥风险；以本地实际工具为准。

如果 Fake GPS 原实现不在本地仓库，应新增明确的 adapter 契约文档和可替换的模拟输入（仅供 SITL），不要悄悄把 ExternalNav 充当 Fake GPS，也不要假装实机路径已验证。

## 4. 第三步：实现 V0 业务逻辑

优先在 `src/indoor_nav` 现有包中增量开发，而不是新建 catkin 工作区。建议采用下列职责（名称可按真实项目习惯调整）：

- `scripts/relative_mission_manager.py`：读取/校验 YAML、等待人工启动、健康与模式检查、冻结原点、坐标转换、单点控制出口、航点/停留状态机、任务事件、正常结束/中止；如原 `mission_manager.py` 模块化优秀，也可以扩展模式而非复制。
- `scripts/v0_nav_health.py`：轻量导航健康监测或复用既有安全模块的**通用**子集；观测真实 `/mavros/state`、`/mavros/local_position/pose`、速度、GPS/EKF 状态（只有确认类型后才订阅）；必要时对接上游 UWB 健康。不得在任务无所有权时乱改飞控模式。
- `config/mission_v0.yaml`：全部任务参数与安全限制，用户未来可直接修改；包含严格 schema 验证。参考随附 `mission_v0_copter.yaml`，但最终代码/文档必须一致。
- `launch/indoor_nav_v0.launch`：**只启动导航节点，不包含 `mavros/apm.launch`、`roscore`、旧 `world_estimator`、GPS 注入节点、传感器录包节点**。有 `enable_control:=false` 或 `dry_run:=true` 的只读联调模式，真机首次默认不发送航点。
- `tools/test_v0_relative_waypoints.py` 和单元测试；独立的 SITL V0 启动入口（复用现有工具，不覆盖旧入口）。

### 4.1 配置坐标约定

用户可编辑 YAML，航点单位**米**：

- `x` 为**任务启动时机头方向向前**；
- `y` 为**任务启动时机头方向向右**；
- `z` 为**任务启动时所在高度向上**；
- 坐标相对整个任务唯一原点 `P0`、唯一初始 `yaw0`，**不是前一个航点累加，也不是随转向旋转**。

ROS/MAVROS local pose 为 ENU，飞控内部 NED 由 MAVROS 插件负责转换；业务层不要再次手写 NED 轴交换。将输入 `(f,r,u)` 转成 FLU `(f,-r,u)`；

```
x_target = x0 + f*cos(yaw0) + r*sin(yaw0)
y_target = y0 + f*sin(yaw0) - r*cos(yaw0)
z_target = z0 + u
```

`yaw0` 从有效的 `geometry_msgs/PoseStamped` 四元数提取；航点 `yaw` 默认保持初始机头角；规范化 quaternion、处理角度跨 ±π、拒绝 NaN/Inf、零范数四元数。对转换函数编写纯 Python 单元测试：初始 yaw=0、+90°、-90°、180°、航点序列原点固定、yaw 改变但任务轴不变、Z 方向不反。

**注意** `z=0` 只是维持**开始任务时的 EKF Z**，不等于距地面零高度。定位源在飞行中重定位/切 EKF 原点时要中止，禁止重新捕获原点后偷偷继续任务。

### 4.2 MAVROS 接口契约

默认：

```
subscribe  /mavros/state                 mavros_msgs/State
subscribe  /mavros/local_position/pose   geometry_msgs/PoseStamped
subscribe  /mavros/local_position/velocity_local (或实际可用速度话题；检查 rostopic type)
publish    /mavros/setpoint_position/local geometry_msgs/PoseStamped
service    /mavros/set_mode              mavros_msgs/SetMode （仅按用户授权的模式策略）
service    /mavros/param/get             mavros_msgs/ParamGet （只读速度/安全参数核验）
```

不要依赖一个臆造的话题名或固定 ROS 插件行为；用 `rostopic list/info/type` 以及真实源码确认。所有控制由**唯一一个 V0 发布者**送到 MAVROS，不允许 mission / watchdog 双写 `/mavros/setpoint_position/local` 或 `/mavros/setpoint_raw/local`。若必要可做 publisher-exclusive gate，但 V0 优先简洁可靠。

按 5–10 Hz（可配置）定期发布**完整、有效、frame_id 一致**的 ENU 位置目标。为避免切到 Guided 的瞬时跳点，先通过只读检查准备状态，锁定当前位姿后先发布当前位姿目标，再在显式授权和模式回读正确后执行相对移动；真实顺序以 Copter 4.5.7/目标固件 SITL 观测为准。**不能以 ROS `publish()` 成功当作飞控确认执行**；必须观测 mode、EKF local pose、目标回显（若有）及到达趋势。

V0 **禁止**通过 `/mavros/cmd/arming`、`/mavros/cmd/takeoff`、`/mavros/cmd/land` 自动实机 ARM/起飞/降落；SITL 测试脚本可在显式 `--sitl` 隔离条件下使用专门受控程序启动仿真飞机，绝不能被实机 launch 意外调用。

### 4.3 速度控制不是 YAML 的魔法

`speed_xy_mps: 0.2` 只有在**确实实现并验证**飞控限制时才算生效。按**实际 ArduCopter 固件**核查 `WPNAV_SPEED`（一些旧版本以 cm/s 计）、`WP_SPD`（新参数以 m/s 计）或 Guided 速度命令 `MAV_CMD_DO_CHANGE_SPEED` 的支持与实际效果，注意参数命名**和单位都可能变化**。

建议 V0 首先只读检查飞控已经人工设好的保守速度/加速度限制；无法识别参数或限制不满足时拒绝启动。需要按航段设置速度的功能可以作为独立可验证扩展，不允许在真机静默修改 FC 参数。记录实际 `/mavros/local_position/velocity*`，加入持续超速警告/中止阈值（区分测量短时尖峰）。

### 4.4 安全、状态机和任务所有权

默认：`IDLE → WAIT_FOR_READY → CAPTURE_ORIGIN → HOLD_INITIAL → EXEC_WAYPOINT[i] → DWELL → FINISH_HOLD → STOPPED`；异常为 `ABORT_LATCHED`。显式 `start`/`abort` 接口可采用 `std_srvs/Trigger`：`/v0_nav/start`、`/v0_nav/abort`；其中启动要求手动 ARM、飞手已悬停、定位和姿态稳定、用户明确授权、模式策略满足、速度参数和全航点软件边界已校验。完成/中止后必须用户显式重新授权，**不自动续飞**。

遇到以下任一问题，取消后续航点并输出故障事件：

- FCU/MAVROS 断链、飞控 mode 非授权 GUIDED、DISARM；
- pose 不更新、header 时间戳过旧/异常、NaN/Inf、位姿跳变、导航质量不合格；
- 上游 UWB Fake GPS 明确失效、EKF GPS/位置不健康（如可获得）；
- 航点超时、离起点越界、持续超速、异常高度、任务节点故障；
- Watchdog 发现任务心跳超时、双控制发布者、ROS clock 异常。

若失效的是**定位本身**，不得把“发送当前点保持”称为安全策略；必要时按 SITL 与拆桨/安全场地已验证的飞控备用动作和飞手接管处理。优先保留飞控侧 failsafe。导航节点死机/MAVROS 断链后 watchdog 未必还有通信能力，不能宣称软件能兜底一切。用户一旦通过 RC 切走 GUIDED，任务立即退出、不得再切回 GUIDED。绝不以室内默认 RTL 处理定位丢失。

到达判定：水平 `sqrt(dx²+dy²)`、Z 误差满足阈值并连续满足 `stable_time_s`，然后启动 `hold_s`，否则直到超时中止。提前验证 YAML 中重复 ID、缺字段、负时间、越界、非法速度、极端位置、零航点等；要求启动前拒绝，而非飞行中才发现。

### 4.5 日志与声学实验对应

新增在 `/v0_nav` 命名空间下的 topic（内容和消息类型统一定义在 README）：

```
/v0_nav/status       std_msgs/String （状态、mission_id、wp、原因；可以用JSON字符串）
/v0_nav/event        std_msgs/String （结构化事件，含wall/ROS stamp、monotonic delta、ID、序号）
/v0_nav/origin_pose  geometry_msgs/PoseStamped
/v0_nav/target_pose  geometry_msgs/PoseStamped
/v0_nav/heartbeat    std_msgs/Header 或 std_msgs/UInt32
```

保存加载 YAML 的绝对路径、SHA256、Git commit、当前工作树是否 dirty、FCU 固件版本/速度参数快照、导航源类型；保存实际发送目标的 topic 和开始/到达/停留/超时/中止事件。不要高频把大 JSON 打印到 SD 卡影响音频。用 ROS 时间记录事件以便和 rosbag 对齐、`time.monotonic()` 做超时与工作周期检测（注意仿真可暂停 `/clock`，要分别处理测试）。

## 5. 与现有声学采集双 ROS 并行（最高优先级约束）

现有附件的 `all_sensors.launch` 包括 `rs485_driver_sound/vibration_node.py`、`xvf3800_node.py`、一次 `mavros/launch/apm.launch`、`auto_bag_recorder.py`；FCU 默认 `/dev/ttyACM0:115200`，MAVROS 提供 ESC/IMU/local pose/state。`auto_bag_recorder.py` 订阅 `/mavros/state`，ARM 自动启动 `rosbag record`，DISARM 发送 SIGINT 结束。它不能被航点控制任务启动/停止进程影响。

**你可能还会在实际机载采集工作区发现新 6×IM72D128 麦克风采集节点**，与旧附件不同。必须尊重机载**最新版本**；不得用旧 `all_sensors.launch` 覆盖新采集系统，不得删除 `/motor_mic/audio_raw`、ESC、XVF raw、振动等话题；不要随便改 16kHz XVF/48kHz IM72D128 的采集协议、分块、增益。

运行约定：

```
ROS Master（1份）
└─ MAVROS（1份，唯一拥有 FCU 串口）
   ├─ 采集业务：rs485_driver_sound + 自动 rosbag
   └─ 导航业务：indoor_nav_ws/V0 + YAML + Guided setpoint
```

两套业务可独立启动/停止；可以先由**当前机载最新的** `all_sensors.launch` 提供唯一 MAVROS；导航 `indoor_nav_v0.launch` 只启动导航节点。SITL 运行时由**唯一 SITL 启动入口**启动模拟 MAVROS，导航 launch 同样复用。两个 shell 的 `ROS_MASTER_URI` 一致，ROS 环境按实际工作区 overlay，避免同名包、不同 ROS master、`/use_sim_time` 污染真机。

只允许对旧采集 launch 做**最小且经用户确认的录包话题扩展**，优先采取可配置 `topics` 添加 `/v0_nav/...`（如果修改会有风险，可提供差异补丁，不擅自覆盖机载版）。导航进程 **绝不**调用、停止或 respawn `rosbag` 和麦克风。导航正常退出后，原采集系统应继续工作；单一声学节点异常时不应自动停止导航任务（但若系统资源损害 pose/控制健康，则按导航健康门槛安全中止）。

需要做的并行验收：

1. 只启采集时，行为不变、rosbag 正常；
2. 只启导航业务并连接唯一 MAVROS（SITL）时，航点正常；
3. 两业务同时运行时，topic 不冲突、定位/控制延迟正常、音频保留完整；
4. 任务中 kill 导航节点：采集/录包不中止，FCU 根据验证的安全策略行为；
5. kill 单个音频节点：不触发不合理的 Guided 重新启动/随意切换；
6. 停止 MAVROS/断连：两个业务都识别，导航进入 latch abort，RC/FCU failsafe 为最后防线；
7. 确认一个 ROS Master、一个 MAVROS 和单一控制发布者（检查 `rosnode`、`rostopic info`、`lsof`）。

## 6. SITL 必须分级测试，尽量利用既有环境

### F0：旧 ExternalNav 回归

不先重跑大量边缘历史实验；确认历史 `run_06` 及基本功能未退化，跑既有 smoke/unit/test 与必要最小回归。**保持旧配置/EKF 源**独立。

### F1：默认 SITL GPS + V0 控制算法

复用当前 ArduCopter SITL/Gazebo/MAVROS 环境，用 SITL 内置 GPS 作为位置源验证**航点算法本身**：

- 用户启动/中止授权；锁定 P0/yaw0；正前 0.2–0.3 m，右侧 0.2–0.3 m，四边形，返回起点；
- 改变机头 yaw 后相对任务轴不漂移；Z 方向测试；
- 到达阈值需持续满足；超时；GUIDED 模式丢失、RC 人工接管后不恢复；
- 配置拒绝非法点；正常 Finish 不自动 ARM/降落；
- 速度参数/命令在 FCU 中真实生效，非仅数值存在 YAML。

对每次测试输出可机器读取的 pass/fail 和：实际位姿、期望目标、Gazebo 真值、姿态/速度、任务事件、飞控模式、起止时间、关键参数、bag/日志路径。

### F2：Fake GPS 等价链路集成（重要，不能用 F1 冒充）

确认真机 Fake GPS 的真实注入协议后，在 SITL 用**同等接口**模拟 `UWB x/y → 虚拟 GPS → FCU EKF3`；不要复用旧 `/mavros/odometry/out` 或 `world_estimator` 来假冒 Fake GPS。隔离 SITL 参数、端口、EEPROM，明确 GPS 来源（通常 `EK3_SRC1_POSXY=3`，以目标固件实际枚举为准），关闭冲突的 SITL 默认 GPS/ExternalNav 输入，验证 EKF 真正采纳注入的 GPS。模拟位置跳变、丢包、停止数据、数据流持续但值冻结、时延、定位噪声、FCU/ROS 断链及状态转换。

如果真实 Fake GPS 的代码/协议暂时不可获得：**不要中断其它代码开发**；交付 `docs/v0/FAKE_GPS_INTERFACE_GAP.md`，写清目前已识别的接口和仍需机载读取的文件/命令，并完成可运行的 F1。把 F2 状态标为 BLOCKED，不得声明已具备真机自动航点放行资格。

### F3：双业务并行压力测试

在 SITL 与可用的声学采集/模拟负载同时运行，关注 CPU、ROS 连接、pose age、setpoint publish jitter、GPS/EKF、rosbag 保存/回放。真实麦克风可能不在台式开发机上：缺硬件可用可信的 topic mock + 负载工具验证通信并在验收中注明，不能替代真机上完整联合测试。

### 真机分级放行（只生成 SOP，不擅自执行）

- R0：**拆桨**，机载实物连接，一次 MAVROS；Fake GPS 数据流、FCU EKF、local pose/yaw、手持前/右/上方向、遥控器模式切换、UWB 断流/跳点测试；禁止自动 ARM。
- R1：由飞手在隔离场地人工低空悬停，先验证现有位置/高度保持、Fake GPS、接管手段；导航 `dry_run`。
- R2：人工授权 Guided，单个 0.2–0.3 m 低速航点，随时 RC 接管。
- R3：小方形/往返，多次重复。
- R4：满负载所有音频/振动/ESC/rosbag 联合测试。

**必须 F0/F1/F2/F3 相应通过才建议开展 R0/R1/R2**；如 F2 BLOCKED，可以进行 R0 的非飞行接口调查，但不得进入自动航点实飞。任何阶段不宣称纯软件安全措施等于物理防护。

## 7. 必需的自动化检查/回归测试

至少交付：

- `pytest` 或 `rostest` 的坐标变换、四元数、YAML schema、原点冻结、到达稳定持续判定、任务状态机、模式切出、双 start、abort latch、stale pose、GPS 跳点与 freeze（避免静止误报）、限速校验/单位转换、时间跳变；
- Launch XML 可解析、catkin build（目标环境）、可正确 `roslaunch` 且无第二 MAVROS；
- 一条 F1 单航点和一条 F1 多航点自动 SITL 记录；
- 如真机 Fake GPS 接口可获得，一条 F2 Fake GPS 集成日志和一个故障注入通过记录；
- 提供不依赖硬件的 `dry_run` / 只读 smoke 测试；
- 源码检查确保生产导航节点没有 `pymavlink` / `mavutil` / `serial.Serial('/dev/ttyACM0')`、没有 `mavros/apm.launch` include、没有自动 ARM/起飞/降落调用。

## 8. 必需交付物与最终汇报

在新分支产出并优先 commit 分阶段稳定状态（不可乱提交用户原有未提交文件，禁止自动 push）：

```
src/indoor_nav/{scripts,config,launch,...}  # 真实落点以项目为准
docs/v0/README.md                     # 安装、运行、配置、控制接口和数据记录
docs/v0/00_REPO_AUDIT.md              # 真实仓库检查结论
docs/v0/SITL_REPORT.md                # F0/F1/F2/F3 实测与证据位置
docs/v0/REAL_AIRCRAFT_SOP.md          # 真机拆桨→人工悬停→短航点→联调
docs/v0/SAFETY_AND_LIMITATIONS.md     # 已知风险、不能证明的安全点
docs/v0/FAKE_GPS_INTERFACE_GAP.md     # 仅当 F2 未打通时必须有
config/mission_v0.yaml               # YAML 位置按真实工程落地
tests/... / tools/...                 # 回归测试和 SITL 工具
```

最终回复务必列出：

1. **真实**当前分支、基线及新 commit SHA、工作区是否干净；
2. 新增/修改文件清单和为什么做；
3. 精确可复制运行命令：F0、F1、F2、dry_run、实机只读启动、stop/abort、rosbag 验证；
4. 测试矩阵的 PASS/FAIL/BLOCKED，与证据路径和失败原因；
5. 如声学 launch 需要新增录包 topic，给出**最小 diff**，不要覆盖机载最新版；
6. 上真机之前必须由人核查的项目：FCU firmware/EKF 及 Fake GPS 来源、模式开关、速度参数及单位、碰撞安全边界、应急程序；
7. 没有完成的工作、已知风险、下一步动作，不得编造日志/结果。

## 9. 参考与优先级

1. **本机真实源码及本机真实实测结果**；
2. 本文配套项目文档和已有历史的 `室内UWB定位与航点飞行_V1方案.md`、声学基线说明；
3. ArduPilot 目标固件版本官方文档/源码；
4. ROS Noetic/MAVROS 对应版本接口。

关键参考：
- https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- https://ardupilot.org/copter/docs/ac2_guidedmode.html
- https://ardupilot.org/copter/docs/common-ekf-sources.html
- https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
- https://docs.ardupilot.org/copter/docs/common-param-name-changes.html

**现在开始：先审计本地 `indoor_nav_ws`，建立不破坏已有改动的新分支，随后编码和跑 SITL。不要停在只给一个设计方案的阶段。**
