# ArduCopter V0 相对航点：实验暂停与审阅报告

> 日期：2026-10-27，Asia/Shanghai。性质：**阶段性暂停快照，不是最终验收或真机放行报告**。
> 最新用户指令：“先暂停实验，生成md报告”。本次仅停止本轮后台实验、读取已有证据和整理本文；不继续开发、不重测、不提交、不操作真机。

## 1. 给网页端审阅者的结论

1. **本轮实验已暂停并清场。** 14:45:34 主机级核验：本轮 SITL、Gazebo、ROS Master、导航及测试进程均不再运行，隔离端口全部释放。两份既有 MAVROS 保留，未误杀。详见第2节。
2. **原四航点“准备悬停超时”已有可解释证据。** 旧驱动复用仿真实例，第二次 ARM 更新 home 后，1m ABOVE_HOME 起飞目标折算为约0.87m EKF高度，不能与 Gazebo 世界高度直接混用。已改为每例独立冷启动，原高度、速度及时间门槛没有放宽。
3. **当前生产源码哈希下，四航点已有独立冷启动 PASS。** 最新证据为 `20261027T143709-multi`，4点完成。但当前同版单点出现 `POSE_FRAME_OR_STAMP`，实际 MAVROS 掉线案例在注入故障前因 `POSE_STAMP_REWIND` 拒绝启动，**不能宣称最终版本稳定全通过**。
4. **控制出口已集中到唯一 gate。** manager 只发任务意图；gate 是唯一生产 MAVROS 位置目标发布者，并管理租约和有条件 BRAKE。生产默认仍为人工接管策略。停止发布、任务锁存或 SetMode 返回成功，均不等于实际刹停。
5. **并行业务验收未完成。** 两个最新 F3 案例均因测试代码读取不存在的 `after_recorder.bag.active` 失败；真实六通道 IM72D128 驱动及机载最新版尚未核实，真实硬件联合验收仍 BLOCKED。
6. **不再有 F2 Fake GPS 专项模拟门槛。** 该项按用户决定为 CANCELLED，不是 BLOCKED。SITL 使用内置 GPS；真实 UWB→飞控 GPS→EKF 的方向、精度和健康状态留给拆桨检查。
7. 工作树不干净，部分安全修复及测试工具尚未提交。**本次暂停不新增 Git 提交，不具备自动航点实飞放行结论。**

## 2. 暂停现场与进程保护

### 2.1 本轮后台批测

- 批测父进程：PID `211140`，启动于13:55:26；证据目录：`docs/v0/evidence/resume-suite-20261027T135526-211140/`。
- 首次主机审计仍看到本轮最后实例 `.sitl_runtime/v0/20261027T144127-266575`；审批/核对期间批测继续执行，因此不能将先前仅完成六例的快照当作最终状态。
- 核验身份后，对批测父进程先发 SIGSTOP，阻止启动下一例。再审计时本轮仿真及测试子进程已经退出，没有对正在飞行的仿真实例强制断电。
- 最后案例的 `cleanup.json` 记录 `armed_after=false`、`connected=true`、`mode=LAND`、`processes_exited=true`；对应 `20-mavros_loss-stop.txt` 记录已请求 PID 限定、DISARM 前提下的关闭。
- 确认子进程消失后，结束被冻结的批测父进程。**没有恢复批测，没有启动补测。**

### 2.2 最后核验

14:45:34 主机级快照保存于：

`docs/v0/evidence/resume_audit/pause_process_snapshot_20261027.txt`

- TCP `11421 / 11455 / 5760`，UDP `9002 / 9003 / 14672`：无本轮监听残留。
- 本轮 suite/test/launcher、ArduCopter、Gazebo、ROS Master、导航节点及 rosbag：快照中均无残留。
- 原有 MAVROS PID `2391064`（10月14日启动）、`3263919`（10月15日启动）：均保留，未操作。
- “本轮隔离 ROS 图内唯一 MAVROS”不等于“整台电脑只有一个 MAVROS”。不得混淆这两种结论。

批测 `results.json` 只落盘20行：17例退出码0、3例退出码1。最后第21例已有独立 FAIL summary 和停机日志，但父进程在完成汇总前被暂停，因此**该批次没有完整的最终总汇总**。不得自行补写成完整成功批次。

## 3. Git、源码与输入身份

### 3.1 实际分支与提交

- 工作空间：`/home/liqidun/github/indoor_nav_ws`。
- 分支：`feat/arducopter-v0-relative-waypoints`。
- 基线：`92f6e724d4ee9ba8c2a348d4ace108e561a4ef30`。
- 原开发提交：`93a5ee0eafbf27b325fb601ee188b40941588bcf`、`a07623f34390065dab960944abf68c4005b5edc5`。
- 当前 HEAD：`3884103860b469c1880c6019bb7a2ede82f27960`，`feat(v0): centralize GPS mission commands in a revocable control lease`。
- 先前尝试的 `fix(v0): deny fallback modes for ambiguous control faults` **没有形成新提交**。本次复核暂存区为空，相关修改仍在工作树。
- 当前 dirty；本次不 commit、不 push、不修改 master。

### 3.2 暂停时 SHA256

| 文件 | SHA256 |
|---|---|
| `src/indoor_nav/src/indoor_nav/v0.py` | `3063e05962fa0064d6519d3499aaa017d5183b7fe121fe13d614a913c1a3ba44` |
| `src/indoor_nav/scripts/relative_mission_manager.py` | `17f3095860900ca237d852fdfd936682c862ed8b69029fecbce3abb44020de4d` |
| `src/indoor_nav/scripts/v0_nav_health.py` | `3ecf17e5da9159952dc141de27895a2627bb30292cfdbee998ecfa902dbc75ff` |
| `src/indoor_nav/config/mission_v0.yaml` | `d00aba3df9e1a0d071f1fdcb2b35a87f9f2247687b0846094ec9ee0601e88ebb` |
| 最新通过四点的独立 `mission.yaml` | `b9659ac4d35ef356299bcab58c91c2cfbf53b1ca8ef4a990be439c1046e23f4c` |
| `src/indoor_nav/config/v0_sitl_gps.parm`（最后案例输入记录） | `965806e915aad5ab94b9096d92e9ec4a6bfbfb51dfdf9387aa4dd4ffa17a6023` |

各案例的 `inputs.json` 记录 Git HEAD、dirty、实际源码/驱动/launch/参数哈希、mission YAML哈希、实例路径和FC参数；导航子目录另有 provenance。**HEAD相同但dirty源码不同，不能视作同一最终版本。**

下文“当前版”仅表示上述三个生产 Python 文件均与暂停时哈希一致，不代表全部测试工具版本一致。长批测中曾修改磁盘工具，已启动的 suite 仍运行其载入时的旧代码，归档时必须继续逐例核对。

### 3.3 未提交内容分类

- 已跟踪文档修改：`00_REPO_AUDIT.md`、`FAKE_GPS_INTERFACE_GAP.md`、`README.md`、`REAL_AIRCRAFT_SOP.md`、`SAFETY_AND_LIMITATIONS.md`。
- 已跟踪生产/测试修改：`src/indoor_nav/scripts/v0_nav_health.py`、`tests/test_v0_gate_ros.py`。
- 已跟踪工具修改：`tools/test_v0_relative_waypoints.py`、`tools/v0_analyze_bag.py`、`tools/v0_sitl_instance.sh`、`tools/v0_sitl_stop.py`、`tools/v0_sitl_test.py`。
- 新增未跟踪报告/证据：`docs/v0/DEVELOPMENT_PLAN_REVISION_20261026.md`、`FOUR_WAYPOINT_ROOT_CAUSE.md`、`SITL_REPORT.md`、`WEB_REVIEW_STATUS_20261026.md`、`evidence/`，以及本报告。
- 新增未跟踪测试/工具：`tests/test_v0_evidence.py`、`tools/v0_archive_evidence.py`、`tools/v0_dataflash_altitude.py`、`tools/v0_sitl_suite.py`。
- 用户原始资料仍未跟踪、未提交、未覆盖：`docs/ArduCopter_V0相对航点_完整项目设计与验收文档.md`、`docs/Codex_开发提示词_ArduCopter_V0相对航点.md`、`docs/README_使用说明.md`、`docs/mission_v0_copter.yaml`。
- 原 ExternalNav 与历史 run_06 保留；没有把历史 ExternalNav 成功当成本轮普通 GPS 结果，没有修改原声学工作区。

## 4. 四航点准备超时：根因与修复边界

详细分析见 `docs/v0/FOUR_WAYPOINT_ROOT_CAUSE.md`，离线数据见 `docs/v0/evidence/resume_audit/historical_altitude.json`。

历史失败：`20261026T182532-multi`，发生于任务 start 前。它与上一轮单点共用 `.sitl_runtime/v0/20261026T181944-3520869`。原始 DataFlash 为该实例 `logs/00000001.BIN`。

本地固件源码 `ardupilot@2a3dc4b7bf2507120f7378a7b2fde73185e0c325`，Copter 4.5.7 的 `do_user_takeoff_start()` 表明，此路径的 takeoff 高度先按 ABOVE_HOME 解释，再换算到 ABOVE_ORIGIN。

| 观测项 | 历史证据 |
|---|---|
| EKF origin | 583.99m |
| 第一次 ARM home | 583.96m，对应1m起飞的EKF目标约0.97m |
| 第二次 ARM 前 CTUN Alt | 约−0.128m |
| 第二次 ARM home | 583.86m |
| 第二次1m起飞的EKF目标 | `1 + 583.86 − 583.99 = 0.87m` |
| 超时前 CTUN DAlt / Alt | 约0.8700m / 0.8595m |
| Gazebo 真值高度峰值 | 约0.83423m，低于驱动0.85m绝对门槛 |

因此，现有证据支持**实例复用与 home/origin/世界真值基准不一致**是该准备失败的关键原因，而不是“飞控收到EKF高度1m却无法飞到”的结论。着陆后的气压/EKF偏差全部成因尚未逐一定位，不作过度因果推断。

已经落地的测试设施修复：每例使用新 runtime/EEPROM，并通过 `flight_claim.json` 阻止同一实例重复飞行。**起飞指令1.0m、真值Z>0.85m、速度范数<0.06m/s、35s超时全部保留。**

恢复后原生产源码的冷启动 `20261026T193632-multi` 已通过原准备门槛，但第四点附近 `POSE_STALE`，pose age约0.420s，仍判 FAIL；没有放宽0.35s门槛。最新当前版 `20261027T143709-multi` 准备时真值Z约0.97951m，并完成4点。二者必须分别报告。

## 5. 实测结果索引

以下每个运行ID均对应 `docs/v0/evidence/<运行ID>/summary.json`；同目录通常有 `inputs.json`、`mission.yaml`、`telemetry.jsonl`、`events.jsonl`、导航日志和 `cleanup.json`。原始 FAIL 保留，不回写为 PASS。

### 5.1 恢复后的早期证据

| 运行ID | 结果 | 说明 |
|---|---|---|
| `20261026T193632-multi` | FAIL | 全新实例通过原准备门槛；第四点附近 POSE_STALE，未根治上游时效问题 |
| `20261026T200142-brake_probe` | PASS | 测试驱动主动BRAKE探针，不能等同生产gate机制完成验收 |
| `20261026T210621-single` | PASS | 初版gate冷启动单点，不是暂停时最终gate哈希 |
| `20261026T210836-multi` | PASS | 初版gate冷启动四点，不覆盖后续失败 |

BRAKE探针 before→after 实际时间约2.32149s；旧summary的 `elapsed_s≈6.847s` 混入后续LAND时间，**不得用作制动时间**。

### 5.2 10月27日已有26份独立结果

当天summary计数：**20 PASS、6 FAIL**，不是最终验收通过率。故障用例PASS只表示符合用例预期，不表示飞机已停稳。

| 运行ID | 结果 | 源码组 | 核心结论 / 失败原因 |
|---|---|---|---|
| `20261027T133310-abort` | PASS | 早期 | 正常abort，显式健康BRAKE策略下约1.752s观察到BRAKE且低速 |
| `20261027T133524-manager_kill` | PASS | 早期 | SIGKILL manager，约2.677s观察到BRAKE低速；首故障为TARGET_OR_ORIGIN_STALE |
| `20261027T133739-pose_loss` | PASS | 早期 | pose转发停流，约0.443s锁存；不请求BRAKE，仍GUIDED且有速度 |
| `20261027T133952-mavros_loss` | FAIL | 早期 | MAVROS死亡触发旧launcher整套退出，重连超时；设施缺陷污染掉线验收 |
| `20261027T134823-mavros_loss` | FAIL | 早期 | 重连后SIM参数未就绪，报not SIM firmware；需专门恢复并降落清理 |
| `20261027T135535-single` | PASS | 早期 | 单点冷启动 |
| `20261027T135802-multi` | PASS | 早期 | 四点冷启动 |
| `20261027T140035-yaw` | PASS | 早期 | 改变初始yaw后的相对航点 |
| `20261027T140309-altitude` | PASS | 早期 | 高度变化/返回路径 |
| `20261027T140528-mode_exit` | PASS | 早期 | 运动中切ALT_HOLD约0.116s锁存；模拟再切回GUIDED也不续飞 |
| `20261027T140743-gate_pause` | PASS | 早期 | gate暂停的检测/锁存；不代表gate自身能可靠执行备用动作 |
| `20261027T140956-manager_pause` | PASS | 早期 | manager暂停，约2.659s观察预期响应 |
| `20261027T141212-concurrent` | PASS | 早期 | 并发控制服务，不接受不合法续飞 |
| `20261027T141427-f3_auto_bag` | FAIL | 早期 | 读取after_recorder.bag.active时FileNotFoundError；真实recorder源码+合成负载，未完整通过 |
| `20261027T141646-distance_short` | PASS | 当前 | 短距离单点 |
| `20261027T141901-distance_long` | PASS | 当前 | 较长距离单点 |
| `20261027T142119-finish_stop` | PASS | 当前 | 完成保持与显式stop流程 |
| `20261027T142344-manager_exit` | PASS | 当前 | manager进程退出场景 |
| `20261027T142601-f3` | FAIL | 当前 | 同样读取不存在的after_recorder.bag.active，完整并行断言未完成 |
| `20261027T142816-link_fault` | PASS | 当前 | 暂停/恢复SITL MAVProxy数据链，约0.438s中止；不等价真实USB拔除 |
| `20261027T143032-gate_kill` | PASS | 当前 | gate被杀后约0.675s观察预期状态；FC仍GUIDED、速度约0.197m/s，**没有刹停保证** |
| `20261027T143247-publisher_conflict` | PASS | 当前 | 冲突publisher仅advertise；约0.222s锁存、不盲目BRAKE，FC仍GUIDED运动 |
| `20261027T143500-single` | FAIL | 当前 | start refused: POSE_FRAME_OR_STAMP；尚未定位根因 |
| `20261027T143709-multi` | PASS | 当前 | 4点完成、原悬停门槛不变，独立冷启动 |
| `20261027T143943-dry_run` | PASS | 当前 | 只读模式；仿真保持DISARM，无运动控制任务 |
| `20261027T144135-mavros_loss` | FAIL | 当前 | start refused: POSE_STAMP_REWIND，未进入实际掉线注入；清理DISARM完成 |

当前三个生产文件哈希下：12份summary，9 PASS、3 FAIL。**早期yaw、altitude、mode_exit、abort、manager_kill等成功结果不能替代当前版完整复测。**

### 5.3 单测与构建

| 项目 | 已有证据 | 边界 |
|---|---|---|
| Python单测 | `resume_audit/unit_tests.txt`，118 tests / OK | 运行在后续小修之前，不是暂停时最终全量 |
| 核心回归 | `resume_audit/core_tests.txt`，9 tests / OK | 旧功能回归证据，非完整ExternalNav重飞 |
| catkin构建 | `resume_audit/build.txt`，此前执行完成 | 后续最终源码未重新全量构建 |
| gate安全小修 | 此前终端单独运行7项通过 | 需后续归档完整终端记录并做最终全量回归 |
| 新证据解析测试 | 已新增 `tests/test_v0_evidence.py` | 不据此推算最终已通过总数 |

本次暂停期间没有为补齐计数再运行任何测试。

## 6. 异常退出机制与剩余风险

### 6.1 已实现的最小控制结构

- `relative_mission_manager.py` 读取任务、冻结原点、产生状态和 `/v0_nav/target_pose` 意图，不再直接发布FC目标。
- `v0_nav_health.py` 为唯一生产运动出口，独立订阅FC pose、速度、GPS/EKF及原点信息；验证租约、消息/目标时效、图独占、围栏和时间异常。
- gate 必须先观察到 IDLE 且 DISARM 才能授予任务控制；不能以空中重启实现自动接管或续飞。
- 人工切出GUIDED立即锁存，不自行切回；生产节点无自动ARM/起飞/降落，无FC参数写入，不管理音频/rosbag生命周期。
- 生产默认 `control.exit_action: manual_takeover`；只撤销控制，不保证飞机停下。
- 显式 `brake_if_position_healthy` 仅在已有任务租约、仍ARM/GUIDED、定位/EKF及4.5.7参数快照健康新鲜的受限条件下，由同一个gate单次请求BRAKE；SITL用例显式启用，真机未获准启用。

最后尚未提交的小修把可BRAKE原因改为明确白名单，防止组合ROS图故障绕过精确字符串黑名单。允许原因仅为空健康原因或任务心跳/状态/目标超时；未知原因、控制冲突、定位坏均不推定可安全BRAKE。该小修发生于批测期间，所以必须按哈希分组。

### 6.2 三类故障必须分开

| 场景 | 当前可做 | 不能承诺 |
|---|---|---|
| 定位正常、manager异常，gate仍活着 | 撤销租约；显式策略满足时请求BRAKE并观测模式/速度 | 不能只凭服务应答称刹停，不能保证所有程序故障可被覆盖 |
| 定位失效/跳变/过旧 | 锁存、停止后续目标，不盲目切依赖位置的模式 | 不保证当前位置保持，不保证BRAKE/Loiter可用 |
| MAVROS完全断连或gate自身死亡 | 由仍存活组件记录/锁存；依赖飞控既有failsafe与RC | 伴随计算机未必仍有通信/控制能力，不能承诺备用动作已执行 |

实例：pose_loss之后仍GUIDED、水平速度约0.209m/s；gate_kill之后约0.197m/s；人工切ALT_HOLD后也可能继续惯性运动。这些场景的PASS是“没有不合法抢控、锁存符合预期”，**不是运动已终止或物理安全已验证**。

gate与manager都观察同一FC EKF；独立检查不是第二套独立定位源。不得将室内默认RTL作为位置丢失策略。

## 7. 速度验证的实际结论

最后案例参数快照含 `WPNAV_SPEED=20`、`WPNAV_SPEED_UP=20`、`WPNAV_SPEED_DN=20`、`WPNAV_ACCEL=50`、`WPNAV_ACCEL_Z=50`；在该4.5.7参数语境下，20cm/s对应期望0.20m/s，**不是已证实的瞬时速度硬上限**。

| 当前版案例 | FC水平峰值 m/s | Gazebo真值水平峰值 m/s |
|---|---:|---:|
| `20261027T141646-distance_short` | 0.20482 | 0.22165 |
| `20261027T141901-distance_long` | 0.21276 | 0.22868 |
| `20261027T143709-multi` | 0.26086 | 0.24726 |

这些是summary在任务活动阶段的采样峰值，不是连续时间上界。最新四点到达真值XY误差约0.0491 / 0.0132 / 0.0188 / 0.0372m；Z误差约0.0017 / 0.0030 / 0.0031 / 0.0075m。

该四点summary记录的最大pose header age约0.0447s，manager周期指标 `max_setpoint_gap_s≈0.10013s`。manager已不是真正FC发布者，此字段不能独自证明实际FC出口抖动；真实gate发布间隔须继续从gate日志/bag分析。

`tools/v0_analyze_bag.py` 已增加>0.20与>0.30m/s累计/最长持续时间、分航点/状态和发布间隔分析，但**最新案例尚未统一离线汇总**。启动/制动/切点阶段的持续超速时间暂不填写，不用峰值替代。未自行放宽物理安全边界。

## 8. 声学并行架构与F3失败

只读找到本地采集仓库 `/home/liqidun/github/onboard_ros_sensor`，HEAD `51be539814869c80b1ebda6c0f9155ff00b03920`；源码身份保存在 `resume_audit/acquisition_source.json`。

- 发现XVF3800、振动、一次MAVROS的 `all_sensors.launch`，以及按ARM/DISARM启停录包的 `auto_bag_recorder.py`。
- 未核实六通道IM72D128驱动，也未证明该仓库就是机载最新部署。没有启动硬件launch、打开硬件串口或修改原采集源码。
- `f3_auto_bag` 使用上述真实recorder脚本连接隔离SITL；传感器是合成PCM大小/序号负载，不能代表真实音频质量。
- 两例F3均在读取 `after_recorder.bag.active` 时失败。现存日志显示recorder曾启动，目录保留 `after_recorder.bag`；**仅凭该文件存在不能证明运行期间持续录包成功**。
- 当前驱动对活动文件路径/时序有假设；确切是启动、命名还是退出竞态，尚待复现和核查，不能先定为音频驱动故障。暂停后没有修改它。

结论：单导航用例已多次观察隔离图的一份MAVROS和唯一gate位置publisher，但“杀导航后bag持续增长”“停bag后导航继续”“高负载下时效与音频完整”整套F3断言**仍FAIL/未完成**；真实麦克风、ESC、全部音频与自动录包联合验收 **BLOCKED（硬件/机载最新版未核实）**。没有覆盖或给机载采集launch施加修改。

## 9. MAVROS掉线失败、恢复与测试设施边界

1. 第一例实际杀MAVROS触发旧launcher的 `wait -n` 退出，连带关仿真。已改launcher只监测本轮核心仿真存活，避免将通信节点死亡等同物理系统消失。
2. 第二例重连connected后过早核验SIM参数，误报not SIM firmware；自动清理未能LAND，一度留下ARM/GUIDED仿真。之后另行确认实例身份、恢复唯一MAVROS、等待参数、LAND至DISARM并关闭。恢复证据为 `resume_audit/mavros_failure_recovery.json`，其中 `disarmed_confirmed=true`。
3. 最新 `20261027T144135-mavros_loss` 在任务start阶段即 `POSE_STAMP_REWIND`，尚未真正执行MAVROS杀进程注入。虽然该例已清理DISARM，**仍不能算实际MAVROS进程死亡/重连验收通过**。
4. `link_fault` 仅暂停/恢复SITL MAVProxy数据路径，已经通过锁存/不续飞测试，不能替代真实MAVROS死亡或真实USB拔除。
5. 当前SITL快照 `FS_GCS_ENABLE=0`、`GUID_TIMEOUT=3`；不能据这些参数或停止位置流就宣称断链自动刹停。真机failsafe策略必须由人独立确认，未改真实参数。

仍需调查：偶发pose header异常/倒退、准备后start时序与MAVROS时间同步/发布回调关系。不能通过忽略倒退、放宽健康阈值或单纯延时来制造成功。

## 10. 证据索引、归档状态与缺口

优先交给网页端阅读：

1. 本报告 `docs/v0/PAUSE_REPORT_20261027.md`。
2. `docs/v0/FOUR_WAYPOINT_ROOT_CAUSE.md`。
3. `docs/v0/SAFETY_AND_LIMITATIONS.md` 与 `docs/v0/REAL_AIRCRAFT_SOP.md`。
4. 当天长批测 `docs/v0/evidence/resume-suite-20261027T135526-211140/results.json`，以及最后一例独立summary，不能只读前20行汇总。
5. 当前版四点 `docs/v0/evidence/20261027T143709-multi/{summary.json,inputs.json,mission.yaml}`。
6. 三份当前版FAIL：`20261027T142601-f3`、`20261027T143500-single`、`20261027T144135-mavros_loss`。
7. `docs/v0/evidence/resume_audit/`：基线、历史高度解析、采集源码身份、MAVROS失败恢复、单测/构建、暂停进程快照。

已有每例原始日志、遥测、事件、YAML、输入哈希及runtime；大bag/BIN/tlog没有在本次暂停中添加到Git。历史失败和旧ExternalNav证据未删除。

`tools/v0_archive_evidence.py` 已新增，但最终统一manifest/test matrix/runtime索引与当前源码匹配的验收汇总**尚未完成正式生成和核验**。本文是人工核对的暂停索引，不冒充最终manifest。suite总结果最后一行缺失如第2节说明；不补写或篡改原始记录。

现有README及部分历史状态说明曾前瞻引用 `RESUME_REPORT_20261027.md`，该文件尚未形成；当前应以本暂停报告为准，不以缺失引用推定已完成最终报告。

## 11. 真机准备文件与放行边界

操作清单已写在 `docs/v0/REAL_AIRCRAFT_SOP.md`；**仅生成文件，没有执行任何真机命令或控制**。

- R0拆桨：确认唯一ROS Master/MAVROS/FCU串口占有者；检查UWB GPS输入与EKF；手持前/右/上方向、yaw与原点捕获；RC GUIDED/AltHold/Stabilize切换、失效时接管；禁止自动ARM。
- 后续人工低空悬停和只读dry_run：由飞手先证明现有定位/高度保持、模式开关和应急可用。
- 首次0.2–0.3m单点及多点：隔离区、物理净空、飞手持续监控与随时接管；速度期望不是硬限，不凭软件围栏保证人员安全。
- 全音频/振动/ESC/自动录包：在前序门槛满足后另行联合验证。

**当前不得仅凭本报告进入自动航点实飞。** 若后续人工授权，可先做R0非飞行接口调查；实际小位移飞行需完成当前版回归、掉线与并行问题闭环，并由人核查FC固件、EKF/UWB质量、速度参数单位、人工模式与应急、碰撞净空、硬件声学负载。

## 12. 恢复后建议优先顺序（目前不执行）

1. 定位 `POSE_FRAME_OR_STAMP`、`POSE_STAMP_REWIND` 与之前POSE_STALE的来源；保留拒绝条件，区分FC/MAVROS时间问题、测试时序与导航逻辑。
2. 查清F3活动bag路径/生命周期失败，再验证导航与recorder独立退出及持续录包，不改原声学协议。
3. 用固定最终源码及工具快照独立冷启动重测：单点、四点、yaw、Z/返回、GUIDED退出、abort、manager/gate故障、实际MAVROS死亡与重连、并发和publisher冲突、只读与合成并行。
4. 汇总峰值与持续超速、实际gate发布间隔、CPU/时效；完成manifest与失败索引；选择性本地提交，不提交用户原始输入和大体积原始包。
5. 人工审阅通过后，再决定是否授权拆桨联调。真实声学硬件联合验证继续明确列为未完成。

**暂停状态保持：不自动恢复实验，不继续开发或提交，等待用户下一步指令。**
