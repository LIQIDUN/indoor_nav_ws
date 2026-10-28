# V0 分级验收实测报告（2026-10-26）

> 历史过程报告：恢复后的最新实测及归档状态见 `RESUME_REPORT_20261027.md`。本文保留旧实现和旧故障记录，不将历史结果改成最终版本PASS。

> **后续范围修订：** F2专项模拟、协议调查及验收门槛已按用户指令取消，不再标为当前开发阻塞；正文F2结论保留为历史过程记录。后续范围见 `DEVELOPMENT_PLAN_REVISION_20261026.md`。实验仍暂停，最终四点失败等既有结果未改变。

> **暂停状态提示（2026-10-26）：** 用户已要求停止实验。本文件为归档进行中的过程稿，部分“manifest/最终归档已完成”措辞尚未实现，F1总结也未充分区分早期通过与最终四点重测失败。请以 `WEB_REVIEW_STATUS_20261026.md` 的暂停快照为准：最终提交单点PASS、四点FAIL、冷启动补测未完成，F2 BLOCKED，不能放行真机。本轮已关闭，未继续实验。

## 总结与边界

**软件交付/F0/F1基础航点及F3模拟负载完成；F2 BLOCKED，不能放行真机自动航点。**

起点 `master@92f6e724d4ee9ba8c2a348d4ace108e561a4ef30`，分支 `feat/arducopter-v0-relative-waypoints`；第一阶段实现提交 `93a5ee0`，最终生产实现及故障工具提交 `a07623f`。后续显式列出原有默认值的YAML和文档/证据随归档提交保存；最终SHA以 `git log -3` 为准。没有push。四个用户原始资料未修改/未提交，因此 `git status` 会保留它们，不能说整个工作区干净。

生产源码SHA随每个导航实例的 `provenance.json` 冻结；`source_coverage.json` 比较最终生产源码与各次实例。前两次接口失败和第三次测试驱动失败全部保留，不回写为PASS。bag/BIN/tlog按既有gitignore保持本地，不强行塞进源码提交；manifest记录hash/大小/真实路径，恢复证据需要同时保留这些本地二进制。

## 测试矩阵

| 层级/项目 | 状态 | 真实证据与范围 |
|---|---|---|
| F0 历史run_06追溯/保护 | PASS | `docs/evidence/sitl_flight/run_06` 保持原hash；历史提交3b3073c，非本轮飞行 |
| F0 单元/旧ExternalNav未解锁回归 | PASS | `evidence/audit/all_tests.txt` 107项；`core_tests.txt` 9项；`f0_calibration.txt` CALIBRATED and LOCKED；`f0_runtime_smoke.txt` 无FAIL，GPS_TYPE=0/POSXY=6/VELXY=0、TF、节点、world_pose等检查通过；没有ACK/ARM/起飞 |
| V0专用单测 | PASS | 上述107项中29项V0测试；含多个参数化subtest：坐标/四元数/schema/原点/稳定持续/锁存/速度单位/时钟/freeze/原点reset/静态禁用API |
| catkin/语法/Launch XML | PASS | `evidence/audit/build.txt`；launch由单测解析；Python编译、bash -n、git diff --check；只安装两新节点，不改旧运行逻辑 |
| 无硬件offline/ROS dry_run | PASS | `offline_smoke.json`、`ros_smoke.json`、`dry_run_launch.txt`；control_enabled=false，无V0控制publisher、无第二MAVROS |
| F1 单航点 | PASS | `evidence/20261026T134856-single/summary.json`，0.25m前向，稳定/停留/完成；随后模式切走锁存、切回不续飞 |
| F1 四航点/yaw/Z/返回 | PASS | `evidence/20261026T142322-multi/summary.json`；前/右/回、+0.1m Z，任务前用仿真yaw命令改变航向；全部4点稳定/停留与真值误差通过 |
| F1 断遥测/恢复不续飞 | PASS（模拟数据链） | `evidence/20261026T143848-link_fault/summary.json`；仅暂停本轮MAVProxy；0.442s观察到POSE_STALE锁存，恢复仍拒绝start；不是实机USB拔线 |
| F2 真实协议等价Fake GPS | **BLOCKED** | 未得到真实协议/端口/程序；`FAKE_GPS_INTERFACE_GAP.md`；没有ExternalNav冒充GPS，也没有编造集成日志 |
| F3 合成音频负载/故障隔离 | PASS（mock限定） | `evidence/20261026T143430-f3/summary.json`；活动任务中kill合成音频不影响导航；活动任务kill manager后音频/rosbag继续，watchdog报告心跳超时 |
| F3 真实采集/ESC/自动ARM录包 | **BLOCKED** | 开发机缺机载最新代码/真实硬件；mock消息为UInt8MultiArray，不等于真实麦克风消息或音频完整性验收 |
| 真机R0–R4 | **NOT RUN** | 仅交付SOP；F2缺失前仅允许R0非飞行接口调查，不建议自动航点实飞 |

### F1 首次通过的实际运动统计

| 指标 | 单点 | 四点 |
|---|---:|---:|
| 启动ENU yaw（约） | 1.579rad（90.44°） | -0.007rad（-0.40°） |
| 最大FC水平实测速度 | 0.26373m/s | 0.26595m/s |
| 最大Gazebo水平真值速度 | 0.24839m/s | 0.24442m/s |
| 到达时最大XY真值误差 | 0.03201m | 0.04989m |
| 到达时最大Z真值误差 | 0.00618m | 0.00561m |
| 活动期最大pose header age | 0.04123s | 0.04416s |
| manager记录最大setpoint间隔 | 0.10008s | 0.10011s |

FC参数只读快照为 Copter4.5.7、WPNAV_SPEED=20cm/s、UP/DN=20cm/s、ACCEL/ACCEL_Z=50cm/s²、GUID_OPTIONS=0、EKF3/GPS源。实际峰值低于本轮 `max_speed_xy_mps=0.30`，但**高于0.20m/s**，说明WPNAV参数不是严格瞬时实测速度上限。不能宣称已实现任何时刻≤0.20m/s；需要更严格速度实验时应另做控制/固件验证。

真值目标由任务开始的local/Gazebo原点和航向差建立固定刚体映射，不直接把Gazebo与EKF的绝对原点当相同；到达判断本身用FC local pose，并独立检查Gazebo真值。`telemetry.jsonl`保留pose/target/truth/velocity/mode/echo/ROS及wall stamp；`events.jsonl`保留到达/停留/锁存时间。`bag_analysis.json`给每航点全过程跟踪误差RMS/最大值（包括移动过程，不把它冒称稳态误差）、事件时间、消息数/间隔；`bag_info.yaml`/`bag_check.txt`及完整逐条读取证明确认包可读。未向实机或活动控制Master回放控制话题。

### F3 实际统计

- 合成XVF块19208字节（19200 PCM等效负载+8字节序号），IM72块86408字节（86400+8），10Hz；格式仅为通信负载，不是ADC采样内容。
- 两音频topic各233条，epoch内序号缺失0；人为kill/restart导致1次序号重置和最大约2.890s断口，这是预期故障，不是“音频全程无缺失”。
- pose约24.983Hz，最大间隔0.04691s；setpoint约9.993Hz，bag中最大间隔0.10056s；活动期pose header age最大0.04274s。
- manager被kill后两秒bag仍增长3,094,721字节，音频进程与录包进程均存活；watchdog明确报告MISSION_HEARTBEAT_TIMEOUT。
- 仿真FC仍ARM/GUIDED，不发生watchdog自主刹停。测试驱动随后显式LAND至DISARM；这是**测试程序**，不是生产导航自动降落。
- 观测的导航进程树RSS峰值约142.3MB；系统CPU采样峰值97.1%（系统总体，不是导航独占CPU，不是硬实时WCET）。满载下本次未触发pose/控制周期期限。
- ROS图归档显示只有一份 `/mavros`，唯一位置控制publisher为 `/v0_nav_manager`，watchdog无控制publisher。只采集窗口由manager kill后的运行窗口覆盖；不能替代真实采集系统的ARM/DISARM触发验证。

## 失败、修正与未测范围

1. `20261026T133336-single`：START拒绝VELOCITY_FRAME_OR_STAMP。现场velocity_local为ENU但header为base_link；下载对应1.20.1官方插件源码核实，无重复旋转；针对这一具体话题允许已审计标签，新增回归。
2. `20261026T133905-single`：START拒绝EKF_STALE。Copter发送msg193，而通用EstimatorStatus依赖msg230；改为从现有MAVROS转发接收并校验msg193，绝不将静默话题当健康。
3. `20261026T134629-single`：任务已FINISH_HOLD，但驱动验证模式切走锁存后，将预期ABORT误当作后续切回GUIDED等待的错误。修正驱动断言后完整重测，原FAIL保留。
4. 最初F0 smoke在沙箱因socket权限失败，初次F0启动器又因rosparam将前导`--`值识别成选项退出；分别保留 `f0_smoke.txt` / `f0_runtime_smoke_initial.txt`。下一次F0局部检查通过但完整日志仍有GPS-disabled情况下缺少EKF原点，记录在 `f0_runtime_smoke_missing_origin.txt`，没有回写PASS。最终按旧流程显式设置固定仿真原点并读回（`f0_origin_prepare.txt`），新实例的完整smoke退出码为0；本报告PASS只指最后一次。
5. 初版GPS启动器停止时先关Gazebo，SITL在FDM接收阻塞且不处理SIGINT；已在确认DISARM后对**本轮特定PID**作TERM/KILL并保留原日志。最终启动器先退出SITL，再清理ROS/Gazebo子进程，并有仅限自己SITL的TERM→KILL兜底。不要把此进程清理策略移植为飞行安全策略。
6. 最终源码复测的 `20261026T182532-multi` 连续复用上一次实例，起飞准备真值最大0.834m，未达驱动0.85m悬停准备门槛，35s超时；此时任务仍IDLE、未发送导航目标。没有降低门槛或改写FAIL；改用全新冷启动独立重测。该现象的深层飞控起飞/高度状态原因未做因果验证，不将推测当结论。

尚未做动态穷举：飞行中主动改yaw同时执行航点、180°初始yaw实飞（四方向已有纯数学单测）、ROS时钟暂停/跳变实飞、真实GPS/UWB冻结/噪声/跳点和EKF内部reset注入、实际MAVROS进程kill/硬件拔线、双控制publisher动态注入、真实音频/ESC/自动录包。对应静态/纯状态机测试不能冒称这些动态验收通过；扩展矩阵尚不完整。

## 证据与复现

精确命令见 `README.md`。所有测试目录均带独立YAML和summary，实例目录带加载YAML/hash/Git/dirty/固件/参数/源码hash。共享仿真DataFlash/tlog跨度覆盖多次测试，原位置及hash列在 `evidence/runtime_index.json`，按每次summary的wall窗口和bag/FC时间对齐，不把整个共享日志当单次独立飞行。大二进制保持本地，不自动push。

`evidence/manifest.json`用于字节校验；`audit/baseline_after.txt`仅允许CMakeLists安装新节点和package.xml依赖两项旧跟踪文件变化，全部旧源码/launch/config/已跟踪历史证据字节未变；四份用户资料hash全部通过。历史被忽略的bag/BIN/旧EEPROM本轮未写入，但没有写前完整hash清单，不把这一点夸大为其逐字节前后hash验证。最终进程/端口检查和本地提交记录以交付时审计结果为准。

最终 `audit/final_ports.txt` 为空，本轮SITL/导航/负载均已关闭。`audit/final_processes.txt`仍列出两份2026-10-14/15启动的旧MAVROS（2391064/3263919），它们连接旧11311的CLOSE_WAIT、UDP14550，未占本轮端口；保持不动。不可把本轮ROS图中的单一MAVROS说成整机全局单一；部署实机前仍须人工消除实际同一部署的冲突。
