# V0-Research 升级与机载交接（2026-11-03）

## 1. 权威基线与工作范围

- 工作区：`/home/liqidun/github/indoor_nav_ws`。
- 起始分支：`feat/arducopter-v0-relative-waypoints`，提交 `dcf3919`。
- 未新建分支、未提交、未pull/reset/clean/push；本次环境 `.git` 只读，直接保留工作区增量。
- 起始存在未跟踪中文设计文档、`docs/mission_v0_copter.yaml`、五份开发/部署/人工审核文档及20261028证据目录。
  全部以SHA256清单保护，本次不修改。精确清单及校验见本次验证记录。
- 已完整审查 v0.py、两个节点、严格配置、导航Launch、CMake/package.xml、核心及gate测试和三份运行/安全文档。
- 不操作机载电脑、真实FCU、声学系统、驱动、固件或飞控参数。历史SITL报告和失败证据不改写。

目标是在最小控制联锁不变的前提下，减少科研实验被悬停噪声、普通速度波动和收敛等待打断。
不是工业级冗余系统，也不构成真实飞行安全认证。

## 2. 策略对照与配置

| 机制 | 严格 V0 / level | research | 是否仍会中止 / 原因 |
|---|---|---|---|
| 软件悬停历史 | require_stable_hover=true；不足2秒或漂移/速度/yaw超限拒绝 | false；不调用严格历史检查 | 本项不阻断；基础健康仍须满足，飞手仍须确认悬停 |
| 持续超速 | overspeed_action=abort | warn | 本项不中止；每任务一次告警，保留当前速度/峰值 |
| 航点等待超时 | timeout_action=abort | warn_and_wait | 本项不中止；每航点一次告警，不跳点、不重置期限 |
| WPNAV速度/加速度 | speed_enforcement=verify_fcu_limit；固定范围 | record_fcu_limits；只读记录、越界诊断 | 性能范围或性能参数缺失不拒绝；接口读取通信故障仍不伪造快照 |
| GUID_OPTIONS/AHRS_EKF_TYPE/EK3_ENABLE/EK3_SRC1_POSXY | 强制匹配 | 不变 | 是；目标接口/GPS定位来源必须可解释 |
| 固件/身份/快照时效 | ArduCopter4.5.7；周期只读核验 | 不变 | 是；未知版本不假设已审计 |
| POSE_JUMP | 单次位移>0.35m锁存 | 不变 | 是；可能破坏已冻结P0，没有独立证据支持放宽 |
| POSE_VELOCITY_FREEZE_INCONSISTENCY | 持续窗口不一致 | 不变 | 是；并非单次速度尖峰，可能已无有效位置参考 |
| EKF_ORIGIN_CHANGED | 原点变化锁存 | 不变 | 是；不重捕获P0偷偷续飞 |
| GPS_EKF_UNHEALTHY / GPS/EKF接收超时 | 拒绝/中止 | 不变 | 是；不能仅靠一帧fix或常量健康信号放行 |
| ROS时间归零/回退、周期异常 | 清旧接收状态/锁存 | 不变 | 是；旧命令不能被当成新参考 |
| RAW_STAMP_* | 仅诊断，以真实回调接收时间判断新鲜度 | 不变 | 原始头诊断本身不终止；不改写头、不用旧位置补发伪造新鲜 |
| FCU连接/ARM/GUIDED、消息/目标有效性 | 必须满足 | 不变 | 是；DISARMED、非GUIDED不能start |
| 目标/实际P0围栏、最大倾角 | 必须满足 | 不变 | 是；不裁剪越界目标 |
| 发布者竞争、Manager/Watchdog失联、租约撤销 | 锁存撤权 | 不变 | 是；人工切模式优先，不抢回GUIDED、不自动恢复 |

旧YAML未写新字段时采用严格默认值。新策略按功能字段选择，不按文件名或全局research标志分叉。
XY/Z容差、stable_time_s、hold_s、max_waypoint_time_s均仍由YAML配置；达到连续稳定和停留条件才推进。
科研版超时等待期间仍执行每周期所有基本健康、空间、控制联锁，也可随时abort或由飞手接管。

三份配置独立可选：

- `mission_v0.yaml`：原始严格基线，字节不变，正右向方形。
- `mission_v0_level.yaml`：严格门控，按本次交接重建负右向（左向）定高方形；不是从机载磁盘导出的历史原件。
- `mission_v0_research.yaml`：相同左向定高航点，四项科研策略明确配置，独立mission_id。

两份新YAML的z均为0。P0=(0,0,1)、yaw0=0时目标是
`(0,0,1),(.3,0,1),(.3,.3,1),(0,.3,1),(0,0,1)`。
**交接文字中的历史测试输出第三点为(.3,-.3,1)，与其给出的负右向YAML及当前已验证FRU→ENU变换不一致。**
本次不为迎合该输出修改坐标变换；负右向是左向，yaw0=0时ENU y为正。
部署前必须从机载实际文件/原点航向/测试脚本核对该方向差异；不能声称完全复现历史任务。
历史SHA256 `887a68cf8dce66624d71d86814cc7bb2125a79788355847ffea1b22cc115cc7e` 不适用于新建文件。

## 3. 仍保留的联锁与职责

Manager只计算相对P0/yaw0的目标，记录任务/参数快照与告警；Watchdog仍是唯一MAVROS位置控制出口。
两个节点读取同一任务、共同调用Health；gate检查YAML哈希/控制开关一致，不接受不同策略的Manager。
Watchdog不自行复制或重新解释超速/航点推进逻辑，故不会对研究策略作相反否决；定位保护始终独立执行。

成功start立即冻结P0/yaw0，然后HOLD_INITIAL；重复start不能改变原点，机头后续旋转不改变任务坐标轴。
围栏是P0周围半径0.80m/垂直±0.20m的示例区域，检查目标及实际位置，不是房间墙壁或完整碰撞检测。

四元数先归一化，原有倾角几何判据保持：`qx²+qy² > sin(max_tilt/2)²`，
等价于 `Rzz=1-2(qx²+qy²) < cos(max_tilt)`，检查机体z轴相对竖直方向的夹角。
纯yaw不影响；最大倾角仍0.35rad（约20°），roll/pitch/组合倾斜和非法四元数均有测试。
本次发现恰好0.35rad时归一化舍入可误报；只对上述平方量添加绝对1e-12数值等价容差，
不扩大实际飞行门槛。精确边界/缩放四元数允许，而0.350001rad仍拒绝。

ControlLease、manual_takeover和条件化brake_if_position_healthy不变。
停止发布不等于刹停；飞控可能继续跟踪最后目标。只有既有租约且定位等条件健康时才允许可选单次BRAKE请求，
定位失效、人工退出GUIDED等不得无条件BRAKE，不自动ARM/起降/抢模式/续飞，不修改飞控failsafe。
节点须在地面DISARM时启动让gate观察IDLE；半空重启不自动获得控制权。

## 4. 可观测性

- `/v0_nav/event` / `events.jsonl`：WARNING为独立事件，不伪装成任务状态转换。
- `/v0_nav/status.warnings`：本任务累计告警，超速最多一次，超时每个航点最多一次；不是当前故障或当前超速位。
- `speed`：最新有效velocity回调对应的XY/绝对Z/三维范数。
- `speed_peaks`：从节点启动或显式reset起所有有效速度回调的最大值；断线不清峰值，reset清除。
- `fcu_parameters` / `performance_diagnostics`：当前只读参数快照及超出严格性能范围的诊断。
- FCU_PARAMETER_SNAPSHOT仅在快照内容变化时记录，不每两秒重复刷屏；通信/接口失效仍反映gate或reason。
- 原有pose_age_s、limit_age_s、接收时刻、原始stamp诊断与EKF flags仍在。

期望巡航、软件超速诊断阈值和实际硬件速度极限不是同一个概念；本系统不输出速度插值器、不自动改PID/WPNAV。

## 5. 机载现场功能集成

新增 `indoor_nav_real_v0.launch` 默认USB `/dev/ttyACM0:115200`，可覆盖fcu_url/gcs_url。
导航入口不隐式启动MAVROS。外层通过start_mavros控制唯一MAVROS include，已有采集MAVROS必须设false。
mission_file完整传至Manager和Watchdog；默认严格level，科研任务只能显式选择。
enable_control=false、dry_run=true；启动Launch本身不启动任务、不ARM/起降。

`v0_request_streams.py` 是用户提供的现场等价实现的本地维护版本，未经机载磁盘逐字节比对。
它复用 `/mavros/set_stream_rate`，首次连接等2秒后请求、服务异常5秒重试、断线重连重新请求。
采用monotonic调度及墙钟等待，不依赖ROS时间持续前进；连接代次避免旧请求完成误标新连接已配置。
不新开MAVLink串口、不依赖pymavlink、不发送模式/ARM/目标；0Hz表示跳过而非关闭FCU流。

| 组 | 已用本地Noetic生成类型核验的ID | Hz |
|---|---:|---:|
| EXTENDED_STATUS | 2 | 5 |
| POSITION | 6 | 10 |
| EXTRA1 | 10 | 10 |
| EXTRA3 | 12 | 5 |
| RAW_SENSORS | 1 | 10 |

现场曾只有HEARTBEAT而部分数据不更新；据用户交接，明确请求这些组后恢复，EXTENDED_STATUS帮助恢复raw/fix。
这不是本次真实硬件复测结论。请求uint16频率校验0..65535，脚本可执行且列入catkin_install_python。

**历史错误**：StreamRateResponse没有success属性。本地ROS Noetic类型的响应slots为空，代码不读取返回对象的success。
ROS服务异常视为未完成，允许重试；正常返回只证明请求调用完成，不能证明FCU接受或实际频率达标。
服务无确认语义，也不能检测连接标志未变化的所有内部遥测丢流；实际话题频率和接收超时仍必查。

## 6. 逐文件变更与部署合并清单

机载“已存在”仅来自用户交接，未访问机载核实。所有覆盖前都须实际diff；未跟踪本地原有文档不属于本次部署包。

| 文件（相对工作区） | 原作用/本次修改 | 新建 | 机载已有 | 处理/部署 | 通信影响 |
|---|---|---|---|---|---|
| src/indoor_nav/src/indoor_nav/v0.py | 状态机/健康；兼容策略、告警、速度峰值、性能诊断 | 否 | 原版 | 合并/需要 | 不改通信 |
| src/indoor_nav/scripts/relative_mission_manager.py | 任务意图；读取参数按策略分层、快照/状态可观测性 | 否 | 原版 | 合并/需要 | 只读ParamGet |
| src/indoor_nav/scripts/v0_nav_health.py | 唯一控制出口；增加速度诊断，联锁不变 | 否 | 原版 | 合并/需要 | 不增加控制出口 |
| src/indoor_nav/scripts/v0_request_streams.py | 五组遥测请求与重连/重试 | 是 | 据交接已存在 | 对比合并/需要 | 改请求遥测率，不新开串口 |
| src/indoor_nav/config/mission_v0.yaml | 严格原始任务，未修改 | 否 | 已有 | 包内供比较，默认保留机载原件 | 无 |
| src/indoor_nav/config/mission_v0_level.yaml | 严格左向定高参考 | 是 | 据交接已存在 | 核对方向/默认保留现场原件 | 无 |
| src/indoor_nav/config/mission_v0_research.yaml | 科研策略与定高任务 | 是 | 未核实 | 添加/需要 | 无 |
| src/indoor_nav/launch/indoor_nav_v0.launch | 现已支持mission_file等，未修改 | 否 | 据交接已改 | 比较保留有效透传 | 不启动MAVROS |
| src/indoor_nav/launch/indoor_nav_real_v0.launch | USB只读入口与stream节点 | 是 | 据交接已存在 | 合并/需要 | 默认启动MAVROS，已有实例须false |
| src/indoor_nav/CMakeLists.txt | 安装stream节点 | 否 | 已有 | 合并/需要 | 无 |
| src/indoor_nav/package.xml | 声明MAVROS运行依赖 | 否 | 已有 | 合并/需要 | 无 |
| tests/test_v0_research.py | 策略、倾角、坐标、参数单测 | 是 | 未核实 | 本地测试，不必部署 | 无 |
| tests/test_v0_research_ros.py | 双节点一致性/参数读取回归 | 是 | 未核实 | 本地测试，不必部署 | 无 |
| tests/test_v0_streams_ros.py | 实际ROS类型/stream与Launch解析 | 是 | 未核实 | 本地测试，不必部署 | 无 |
| tools/v0_sitl_test.py | 可显式选择任务模板并记录模板哈希 | 否 | 未核实 | 仅开发机，不部署执行 | 仅隔离SITL |
| tools/v0_sitl_suite.py | 透传上述测试模板选项 | 否 | 未核实 | 仅开发机，不部署执行 | 仅隔离SITL |
| docs/v0/README.md | 增加科研/机载入口与配置说明 | 否 | 未核实 | 合并/建议部署 | 无 |
| docs/v0/SAFETY_AND_LIMITATIONS.md | 科研差异及保留定位联锁 | 否 | 未核实 | 合并/建议部署 | 无 |
| docs/v0/REAL_AIRCRAFT_SOP.md | 只读入口、飞手职责/方向提醒 | 否 | 未核实 | 合并/建议部署 | 无 |
| docs/v0/V0_RESEARCH_UPGRADE.md | 本设计/交接/配置说明 | 是 | 未核实 | 添加/建议部署 | 无 |
| docs/v0/V0_RESEARCH_DEPLOYMENT.md | 备份、逐文件合并、增量部署、回滚命令 | 是 | 未核实 | 添加/建议部署 | 无 |
| docs/v0/V0_RESEARCH_DEPLOY_FILES.txt | 明确部署包白名单 | 是 | 未核实 | 添加/建议部署 | 无 |
| docs/v0/V0_RESEARCH_VALIDATION_20261103.md | 本次真实执行的验证结果 | 是 | 未核实 | 添加/建议部署 | 无 |
| docs/v0/evidence/v0-research-20261103/ | 本地测试日志、基线/原有文件校验 | 是 | 不适用 | 本地留存，不要求部署 | 无 |

## 7. 验证与同步边界

准确命令、测试数量、失败尝试、构建/SITL及未测项见 `V0_RESEARCH_VALIDATION_20261103.md`。
SITL工具新增 --mission-file 只选择策略模板，仍替换为对应case航点、仿真源及条件化BRAKE；保留原身份门禁和准备条件。
不能把测试生成任务当作生产五点原样飞行，也不能把仿真BRAKE策略当作现场默认。

开发机本地审查→只提交明确白名单文件→人工push GitHub→机载先备份和diff→合并后构建→只读验证。
具体命令见部署文档；本次不执行提交/push或机载操作。若准备建分支，应由用户在可写.git环境先保护现有未跟踪资料。

未完成真实飞机科研航迹、真实USB长期GPS/姿态/EKF频率、RC带动力接管、failsafe、倾角/围栏触发实飞、采集联合验收。
用户交接中的实机只读状态和历史SHA只能作为历史报告，不提升为本次自动航迹验收证据。
