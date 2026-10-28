# indoor_nav_ws / ArduCopter V0：暂停实验时的结果与现状报告

> 历史暂停快照：用户后续已授权恢复本地代码和隔离SITL。新结果、唯一控制gate及故障复测见 `RESUME_REPORT_20261027.md`；本文旧watchdog行为/未完成项不自动代表当前版本，也不回写原始失败。

> 给网页端审阅的自包含快照。日期：2026-10-26，时区 Asia/Shanghai。  
> 用户要求已切换为“停止实验、整理结果”，本报告整理阶段没有启动新实验或继续开发控制功能。  
> **这不是最终验收报告，不是实机飞行放行文件。** 下文保留失败、证据缺口和未提交状态。

> **后续范围修订：** 用户随后确认UWB经飞控GPS接口提供定位，并取消F2专项模拟、底层协议调查及其验收门槛。本文保留当时审查快照，正文中的F2 BLOCKED/调查建议不再是当前待办；后续以 `DEVELOPMENT_PLAN_REVISION_20261026.md` 为准。已有实验结果和安全限制未因范围调整而改变，实验仍暂停。

## 1. 结论先读

1. **本轮实验已停止。** 安全停止工具在确认隔离 SITL 已连接且未解锁后关闭本轮实例；2026-10-26 19:09:31 +08:00 主机级检查未发现本轮测试、导航、合成负载、录包进程或隔离端口监听。没有操作真机。
2. V0 相对航点代码已经落地，有两个本地提交。离线测试、构建、只读 smoke 有通过记录；旧 ExternalNav 做了未解锁 smoke，没有重飞历史 run_06。
3. F1 内置 GPS 仿真有单点和四点通过记录；**最终提交单点重测 PASS，最终提交四点重测 FAIL，冷启动补测没有形成新的正式通过结果。** 因此不能概括为“最终版本 F1 已全部通过”。
4. **F2 BLOCKED：真实 UWB → Fake GPS 注入程序、协议和端口未取得。** 本轮没有验证真实协议等价链路，不能用内置 GPS 或旧 ExternalNav 结果替代。
5. F3 只证明了合成话题负载下的部分业务隔离；没有验证机载真实 XVF3800、IM72D128、ESC 或 ARM/DISARM 自动录包。
6. 关键安全限制：**停发位置目标不等于刹停；watchdog 只报告、不接管；WPNAV_SPEED=20 cm/s 不等于实测瞬时速度始终 ≤0.20 m/s。** 当前不能放行真机自动航点。
7. 工作树不干净，实验资料尚未最终归档/提交；没有 push。用户原始资料、旧采集系统和历史证据没有被覆盖。

## 2. 本次任务目标与实现边界

目标是在现有 catkin 工作区增量加入独立导航业务：飞手人工 ARM、起飞并稳定悬停，人工进入 GUIDED 后，通过显式服务授权执行 YAML 中相对任务起点的短距离航点。

设计目标部署关系如下；**真实 Fake GPS 上游目前尚未核实**：

```text
UWB → 真实 Fake GPS 上游（待查明）→ FCU EKF3
                                      │
同一次部署：一个 ROS Master、一个 MAVROS、一个 FCU 串口占有者
                                      ├─ 原采集业务 / 自动 rosbag
                                      └─ V0 导航：读已融合 local pose → 唯一位置目标发布者
```

已实现的生产导航不自动 ARM、起飞、降落，不写飞控参数，不自动切 GUIDED，不启动/停止音频或 rosbag。SITL 驱动可在隔离身份门禁及显式 `--sitl --execute` 下操作仿真飞机；它不被生产导航 launch 调用。本轮报告整理期间未执行该驱动。

相对坐标为前/右/上，冻结唯一 P0 和 yaw0，所有点相对任务起点而非逐点累加；转换到 ROS ENU 后交由 MAVROS 处理飞控坐标，不在业务层重复 ENU/NED 转换。`z=0` 表示保持任务开始的 EKF 高度，不代表地面。

## 3. 停止现场与保留的旧进程

停止对象：`.sitl_runtime/v0/20261026T183647-3535222`。

| 对象 | 本轮 PID | 最终核查 |
|---|---:|---|
| 隔离启动器 | 3535222 | 已退出 |
| ArduCopter SITL | 3535364 | 已退出 |
| 本轮 MAVROS | 3535448 | 已退出 |
| gzserver | 3535491 | 已退出 |
| 本轮 rosmaster | 3535296 | 已退出 |

停止前只读状态为 connected=true、armed=false、STABILIZE；停止工具再次要求 connected 且 DISARM，并核对 runtime、SITL 可执行文件和启动器父子关系。实际工具输出：

```text
Requested PID-scoped disarmed SITL shutdown: 3535222 /home/liqidun/github/indoor_nav_ws/.sitl_runtime/v0/20261026T183647-3535222
```

停止后主机级 `ps` 未匹配本轮启动器/测试驱动/导航/mock/rosbag record；`ss -luntp` 未匹配 TCP11421、TCP11455、TCP5760、UDP9002/9003、UDP14672。对应五个 PID 的精确检查也无进程行。该精确 `ps -p` 无匹配时退出码为 1，并非停止失败。

**有两份早于本轮的 MAVROS 被刻意保留**：PID2391064（10月14日启动）、PID3263919（10月15日启动）。因此只能说“本轮隔离 ROS 图中一份 MAVROS”，不能说“整台主机只有一份 MAVROS”。没有为清理本轮实验误杀旧业务。详细停止记录另存 `docs/v0/evidence/audit/pause_shutdown_20261026.md`。

## 4. Git、环境与未提交内容

工作区：`/home/liqidun/github/indoor_nav_ws`。

| 项目 | 事实 |
|---|---|
| 基线 | `master@92f6e724d4ee9ba8c2a348d4ace108e561a4ef30` |
| 当前分支 | `feat/arducopter-v0-relative-waypoints` |
| 第一阶段提交 | `93a5ee0eafbf27b325fb601ee188b40941588bcf`：相对航点、健康门禁、独立 GPS SITL 工具 |
| 当前 HEAD | `a07623f34390065dab960944abf68c4005b5edc5`：EKF 原点变化锁存、故障隔离测试 |
| 工作树 | **不干净**；本次整理没有新 commit，没有 push |
| 已审计环境 | ROS Noetic、MAVROS 1.20.1、Gazebo 11.15.1、ArduCopter 4.5.7，本地飞控源码 `2a3dc4b7bf` |

停止整理前的已跟踪未提交修改：

- `docs/v0/00_REPO_AUDIT.md`：补充审计结论。
- `docs/v0/README.md`：补充命令、结果解释和限制。
- `src/indoor_nav/config/mission_v0.yaml`：把原 schema 的 21 个安全默认值显式写入 YAML；不是新增一轮测试结果，不能把旧 smoke 的 YAML 哈希等同于当前文件哈希。

未跟踪的新增实验文档/证据：`docs/v0/SITL_REPORT.md`、`docs/v0/evidence/`；本次整理再新增本报告和停止记录，并为旧实测报告加暂停状态提示。没有修改生产代码。

以下四份用户输入继续保持原样、未提交：

```text
docs/ArduCopter_V0相对航点_完整项目设计与验收文档.md
docs/Codex_开发提示词_ArduCopter_V0相对航点.md
docs/README_使用说明.md
docs/mission_v0_copter.yaml
```

`evidence/audit/user_inputs_after.txt` 为四份输入的 SHA256 核验通过记录。`baseline_after.txt` 的旧基线文件核验中只有 CMakeLists/package.xml 两项有预期变化；其余旧已跟踪源码、launch/config 和已跟踪历史证据通过。**没有对历史 ignored bag/BIN/EEPROM 建立完整写前 hash 清单**；本轮未写这些旧文件，但不宣称对它们完成逐字节前后核验。

## 5. 已落地文件与职责

下列路径相对工作区根目录；具体运行说明见 `docs/v0/README.md`。

| 文件 | 作用 |
|---|---|
| `src/indoor_nav/src/indoor_nav/v0.py` | ROS 无关 schema、四元数/坐标转换、健康检查、原点冻结、状态机、持续到达/停留、abort 锁存 |
| `src/indoor_nav/scripts/relative_mission_manager.py` | 唯一 MAVROS 位置目标出口；服务授权、状态/事件、固件/参数只读核验、运行来源记录 |
| `src/indoor_nav/scripts/v0_nav_health.py` | 心跳/ROS 图监督，只报告，不另发控制目标 |
| `src/indoor_nav/config/mission_v0.yaml` | 用户可编辑航点与安全限制；默认生产只读联调 |
| `src/indoor_nav/config/v0_sitl_gps.parm` | 独立 F1 内置 GPS 参数，不复用旧 ExternalNav 源配置 |
| `src/indoor_nav/launch/indoor_nav_v0.launch` | 只启动 manager/watchdog；不含 MAVROS、GPS 注入、旧 world_estimator 或采集节点 |
| `tests/test_v0.py` | 29 个测试方法及参数化子测试 |
| `tools/test_v0_relative_waypoints.py` | 无硬件配置/变换 smoke |
| `tools/v0_sitl_instance.sh`、`tools/v0_sitl_stop.py` | 独立 runtime、端口、EEPROM；受限启动/停止 |
| `tools/v0_sitl_test.py` | 仅隔离 SITL 的单点/多点/链路故障/F3 驱动 |
| `tools/v0_mock_acquisition.py`、`tools/v0_analyze_bag.py` | 合成通信负载、离线证据分析 |
| `docs/v0/{README,00_REPO_AUDIT,REAL_AIRCRAFT_SOP,SAFETY_AND_LIMITATIONS,FAKE_GPS_INTERFACE_GAP}.md` | 部署、审计、人工分级 SOP 与接口缺口 |

原有文件仅有 `src/indoor_nav/CMakeLists.txt` 增加节点安装、`src/indoor_nav/package.xml` 增加 geographic_msgs 运行依赖；旧导航实现没有被改写成 Fake GPS。没有修改机载采集 launch。

### 控制与安全语义

- 默认 `enable_control=false`、`dry_run=true`，同时显式开启控制并关闭 dry_run 才允许生产节点发控制目标。
- `/v0_nav/start` 显式授权；`abort` 锁存停止后续点；`stop` 结束 FINISH_HOLD 输出；`reset` 要求连接正常、DISARM、非活动状态，不能自动续飞。
- 状态流程为 IDLE → WAIT_FOR_READY → CAPTURE_ORIGIN → HOLD_INITIAL → EXEC_WAYPOINT/DWELL → FINISH_HOLD；异常 ABORT_LATCHED。飞手切出 GUIDED 后不抢回模式。
- 位姿、速度、GPS/EKF、时间戳、原点通知、边界、持续超速等构成门禁。GPS/UWB 共同冻结不能单靠 EKF pose/velocity 完整识别；真实上游质量仍缺失。
- `/v0_nav/status`、`event`、`origin_pose`、`target_pose`、`heartbeat`、`watchdog` 提供状态与对齐信息；target 预览或 publish 成功不是飞控执行确认。

## 6. 测试矩阵：通过范围与缺口

证据根路径为 `docs/v0/evidence/`。以下均是此前实测/日志核对结果，**不是暂停后重跑**。

| 项目 | 状态 | 证据与边界 |
|---|---|---|
| 历史 run_06 追溯 | 历史记录已核对 | `docs/evidence/sitl_flight/run_06`、提交 `3b3073c`；2026-10-09 ExternalNav 10/10、约53.128s，非本轮重飞 |
| 单元测试 | PASS | `audit/all_tests.txt`：107 项通过（78旧+29 V0）；`audit/core_tests.txt`：另9项通过 |
| 构建/静态检查 | PASS（既有记录） | `audit/build.txt` catkin_make；Python编译、bash语法、Launch XML/禁止API检查有通过记录；不代表全部动态故障已测 |
| 无硬件/ROS 默认只读 | PASS | `audit/offline_smoke.json`、`audit/ros_smoke.json`、`audit/dry_run_launch.txt`；只读下无 V0 控制 publisher |
| F0 旧 ExternalNav 最小回归 | PASS（未解锁 smoke） | `audit/f0_origin_prepare.txt`、`audit/f0_runtime_smoke.txt`；最终完整 smoke 退出0；未 ACK/ARM/起飞，不证明本轮动态 ExternalNav 飞行 |
| F1 早期单点 | PASS | `20261026T134856-single/summary.json`；不是最终生产源码哈希 |
| F1 早期四点/yaw/Z/返回 | PASS | `20261026T142322-multi/summary.json`；4点，任务前改 yaw、上升0.1m及返回；不是最终生产源码哈希 |
| F1 最终提交单点重测 | **PASS** | `20261026T182505-single/summary.json`；三个生产文件哈希与当前一致 |
| F1 最终提交四点重测 | **FAIL** | `20261026T182532-multi/summary.json`；仿真准备悬停超时，导航尚未start，不可用早期PASS覆盖 |
| F1 冷启动补测 | 未完成/无通过结果 | 最后补测日志为 Master/隔离身份检查失败；之后用户中断，现存正式summary中没有更晚四点通过记录 |
| F1 遥测中断/恢复 | PASS（模拟链路限定） | `20261026T143848-link_fault/summary.json`；暂停本轮 MAVProxy，约0.442s出现 POSE_STALE，恢复不续飞；不是实机USB拔线 |
| F2 等价 Fake GPS | **BLOCKED** | 真实协议/实现未知；没有该链路集成、故障注入通过证据 |
| F3 合成负载与故障隔离 | PASS（mock限定） | `20261026T143430-f3/summary.json`；不能当真实音频/ESC/自动录包验收 |
| F3 真实采集联合测试 | **BLOCKED** | 缺机载最新版代码和硬件 |
| R0–R4 实机试验 | **NOT RUN** | 未执行；仅交付 SOP，目前不具备自动航点放行资格 |

单元测试覆盖坐标/四元数/schema、原点不变、持续到达、模式切出/双start/abort锁存、stale pose、跳点/freeze一致性、速度单位、时钟和原点reset等。这里的“覆盖”指纯逻辑/替身测试，不等于这些故障都在飞行中注入验证。

### 6.1 三次通过航点试验的数值

| 指标 | 13:48单点（早期） | 14:23四点（早期） | 18:25单点（最终提交） |
|---|---:|---:|---:|
| 完成航点数 | 1 | 4 | 1 |
| 到达时最大 Gazebo 真值 XY 误差 | 0.03201m | 0.04989m | 0.00964m |
| 到达时最大 Gazebo 真值 Z 误差 | 0.00618m | 0.00561m | 0.00077m |
| FC 水平速度峰值 | 0.26373m/s | 0.26595m/s | 0.25146m/s |
| Gazebo 水平速度峰值 | 0.24839m/s | 0.24442m/s | 0.24013m/s |
| 活动期最大 pose header age | 0.04123s | 0.04416s | 0.04832s |
| manager 最大目标发布间隔 | 0.10008s | 0.10011s | 0.10009s |

真值比较使用任务开始时 local/Gazebo 坐标关系的固定映射，并非假定两个绝对原点天然重合。以上是到达事件对应的误差，不是整个航段的最大跟踪误差，也不是实际 UWB 定位精度。

速度快照为 Copter4.5.7 的 `WPNAV_SPEED=20 cm/s`、UP/DN=20 cm/s、ACCEL/ACCEL_Z=50 cm/s²、GUID_OPTIONS=0。本轮实测峰值低于测试监测门槛0.30m/s，但超过0.20m/s。**如果实验硬要求任意时刻不超过0.20m/s，当前结果不满足。** 参数只读核验/单位换算不等于硬速度保证。

### 6.2 F3 实测与不能推导的结论

- 合成 XVF/IM72 负载分别为19208/86408字节块（含序号前缀），模拟10Hz话题传输，类型为 UInt8MultiArray；不是驱动原始消息、真实 PCM 或 ESC 数据。
- 活动任务中 kill 合成音频节点没有导致导航异常切模式/重启；人为重启产生约2.89s缺口，不能声称全程音频无间断。
- kill manager 后合成采集和 rosbag 继续，bag 增长3,094,721字节，watchdog观测到 MISSION_HEARTBEAT_TIMEOUT。
- **FCU当时仍为ARM/GUIDED，watchdog没有自主刹停。** 测试驱动随后执行仿真LAND/DISARM；不是生产导航自动降落。
- 两个合成音频话题各233条，重启 epoch 内序号缺失0；pose约24.983Hz，最大间隔0.04691s；目标约9.993Hz，最大间隔约0.10056s。
- 导航进程树RSS峰值142,266,368字节；系统CPU采样峰值97.1%，是整机采样而非导航独占CPU，不能推导硬实时保障或真实满载长期可靠性。
- 本机可读的 `ros_sensor` 采集 launch 与用户描述的机载最新版本不一致，未取得可核验的最新自动 recorder/IM72部署。没有覆盖它们，也没有将 mock 当实机联合测试。

## 7. 全部正式失败与最后中断

保留的9份正式 summary 中5份PASS、4份FAIL；它们是不同版本/不同案例的开发记录，**不是统计成功率样本**。

| 目录/日志 | 结果 | 观察、修正与尚不能断言的内容 |
|---|---|---|
| `20261026T133336-single` | FAIL | START拒绝 `VELOCITY_FRAME_OR_STAMP`。MAVROS1.20.1指定 velocity_local 实际为ENU但header标base_link；查对应插件源码后仅对该接口接受经审计标签，没有重复旋转 |
| `20261026T133905-single` | FAIL | START拒绝 `EKF_STALE`。目标Copter发msg193而msg230话题静默；改用已有 `/mavlink/from` 的固定msg193解码/来源与framing状态检查，不新增 MAVLink 连接 |
| `20261026T134629-single` | FAIL | 航点已完成，驱动在模式切出锁存后把预期ABORT当错误；修复断言并另建目录重测，不回写原FAIL |
| `20261026T182532-multi` | **FAIL** | 最终提交、复用前一实例；准备阶段 `timeout: hover`，真值最高约0.834m，未过驱动“Z>0.85m且速度范数<0.06m/s”的35s门槛。导航IDLE、wp=0、max_publish_gap=0；不是四点执行到一半失败。未降低门槛，深层飞控/高度原因未做因果验证 |
| `/tmp/indoor_v0_cold_multi.txt` | 未形成正式案例 | 冷启动补测出现 `Unable to register with master node`、`RuntimeError: not the isolated V0 ROS master`，身份门禁退出；没有新的正式summary，不记为通过 |

F0也有三次保留的前置失败：沙箱socket权限（`f0_smoke.txt`）、启动profile rosparam解析（`f0_runtime_smoke_initial.txt`）、GPS关闭且缺固定EKF原点（`f0_runtime_smoke_missing_origin.txt`）。最后通过显式设置/读回旧仿真固定原点完成 smoke；这里只认可最终完整日志，不引用中途局部通过输出作全量结论。

最后的冷启动续测请求被用户中断。报告整理时没有发现仍在执行的驱动，也没有比18:25四点失败更新的正式summary；因此最终状态为“补测未完成”，不能写成“冷启动已解决”。

## 8. 源码对应关系与证据归档缺口

当前三个生产文件 SHA256：

```text
482cc55d081ccff2639dcb009783523638f08de14fa3bfc7b91e01e8f5be8bd7  src/indoor_nav/src/indoor_nav/v0.py
778bddbd0a05fe59cfb14322dc3530b69b8273e6dd4b816a974467edc8acb635  src/indoor_nav/scripts/relative_mission_manager.py
58c55369b43cdfdfad9e7b3da1525e36826cacb5300a223e4074aeafcd12501b  src/indoor_nav/scripts/v0_nav_health.py
```

源码归属不能只看运行时 Git HEAD：开发时有dirty树，实例记录的 `provenance.json/code_sha256` 更能界定实际文件。

- 早期13:48单点、14:23四点的核心/manager哈希与当前不同，后续增加了原点变化监测。
- F3及link_fault的三个生产文件哈希与当前一致，尽管当时记录的HEAD仍为第一阶段提交且dirty。
- 18:25最终单点及紧接的四点失败都记录HEAD=`a07623f...`、dirty=true；本次逐项核对其 `code_sha256` 与当前三个生产文件一致。此核对不表示整套工具/YAML/环境字节完全相同。

主要证据位置：

```text
docs/v0/evidence/audit/                         审计、测试、构建、F0、只读smoke
docs/v0/evidence/<日期时间-案例>/summary.json    每次案例最终判定
docs/v0/evidence/<日期时间-案例>/telemetry.jsonl 位姿/目标/真值/速度/模式等
docs/v0/evidence/<日期时间-案例>/navigation/    原始YAML、provenance、稀疏事件
docs/v0/evidence/<日期时间-案例>/bag_analysis.json  已完成案例的离线分析（非每个目录都有）
.sitl_runtime/v0/<实例>/                        隔离EEPROM、DataFlash、tlog及进程日志
```

**归档尚未完成，必须明确纠正早先文档中的完成式措辞：**

1. `evidence/manifest.json` 和 `evidence/test_matrix.json` 当前不存在，不能宣称已完成总清单和全量字节校验。
2. `runtime_index.json`、`runtime_text/`、`source_coverage.json` 为较早快照，未覆盖全部较新实例；最新两例源码关系是本次额外只读核对，不是假定旧索引已更新。
3. 较新案例不保证已有 bag_analysis。大bag/BIN/tlog遵循既有gitignore留本地，不等于已随源码提交或可由网页端直接访问。共享DataFlash跨多个案例，不能将其全段冒称单次飞行。
4. `SITL_REPORT.md` 存在归档进行中写下的超前措辞，例如manifest/最终提交及F1总结；本报告在暂停时优先，旧报告仅作详细过程材料。
5. 尚无完整干净交付提交；恢复工作前需区分用户原始输入、代码、文档、文本证据和大二进制，禁止一把 `git add .`。

## 9. Fake GPS 与实机安全尚缺什么

F1用的是SITL内置GPS；旧成功链路是ExternalNav `/mavros/odometry/out`。二者都不能回答真实UWB Fake GPS究竟走GPS串口、GPS_INPUT、HIL_GPS还是其它协议。

必须由机载只读调查补齐：真实程序路径/版本/启动命令、输入UWB消息、输出协议与频率/坐标/单位、高度/yaw处理、FCU端口映射、固件与EK3/GPS/SERIAL参数、原始测量时间戳/序号/质量/reset语义。具体契约见 `FAKE_GPS_INTERFACE_GAP.md`。

当前监测 `/mavros/global_position/gp_origin` 的变化会锁存，但它不覆盖所有EKF内部reset；下游pose/velocity共同冻结也可能漏检。缺少真实上游健康/重定位信号不能通过持续发布常量 `healthy=true` 解决。

尚未完成的动态场景包括：飞行中主动转yaw同时走点、180°初始yaw动态试验、ROS clock暂停/跳变飞行注入、真实GPS/UWB停流/冻结/噪声/跳点及EKF内部reset、实际MAVROS进程kill/硬件拔线、动态双控制发布者注入、真实音频/ESC/ARM录包联合测试。数学/状态机测试不能替代它们。

**在F2和相应联合验收未完成前，不建议R1/R2自动航点实飞。** 只可由人安排拆桨、非飞行接口调查；不能把当前软件中止机制视为物理防护、RC接管或飞控failsafe的替代品。

## 10. 采集业务的处理情况

原采集业务没有被修改；导航不拥有rosbag/麦克风进程。真实机载最新版未取得，因此目前没有可直接套用的精确采集launch补丁。仅保留供人工对照的最小追加示意：

```diff
-<param name="topics" value="原有完整话题列表"/>
+<param name="topics" value="原有完整话题列表 /v0_nav/status /v0_nav/event /v0_nav/origin_pose /v0_nav/target_pose /v0_nav/heartbeat"/>
```

这是示意而非可执行patch，不要把占位文字替换掉原列表。必须保留真实XVF/IM72/ESC/振动话题及采样协议；确认最新recorder确实使用此参数后再由人批准修改。

## 11. 请网页端重点审阅

1. **失效策略是否符合项目门槛？** 当前 abort/kill manager 只是停发，FCU仍可能执行最后目标；watchdog不刹停。应明确经验证的飞控/飞手备用流程，还是需要另行设计受控且单一出口的备用动作。
2. **速度要求到底是什么？** 若0.20m/s是硬上限，当前结果不满足；如只是期望巡航速度，需明确允许峰值、持续超限阈值和物理净空。
3. **最终四点失败应如何界定与排查？** 已知是准备悬停超时，尚非航点逻辑运行失败；不能未经证据把它定性为纯测试问题，也不能降低门槛消除失败。
4. **Fake GPS证据收集优先级是否正确？** 当前先取得真实协议和机载部署，再建等价F2，比继续堆F1飞行次数更能补关键缺口。
5. **版本与验收用词是否足够严格？** 早期多点PASS、最终源码单点PASS、最终源码多点FAIL、F3 mock PASS必须分开；不能汇总成“V0全验收通过”。
6. **后续恢复工作范围：** 先做最终版本独立冷启动四点复测及前置超时排查，再按真实协议推进F2/真实负载F3，最后补完整manifest和有边界的归档提交。以上仅为待审议建议，**本次未执行**。

## 12. 当前冻结状态

本轮仿真已停，旧进程保留，失败记录保留，代码停在上述HEAD及未提交修改状态。没有继续试飞、没有降低测试门槛、没有自动实机解锁、没有修改真机参数、没有覆盖机载采集系统、没有push。等待用户与网页端审阅结论后再决定是否恢复实验。
