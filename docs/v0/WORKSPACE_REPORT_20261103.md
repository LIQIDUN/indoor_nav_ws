# indoor_nav_ws 最新修改与运行记录简报

> 用途：提供给网页端的项目上下文与后续讨论依据。
> 整理时间：2026-11-03，Asia/Shanghai；工作区：`/home/liqidun/github/indoor_nav_ws`。
> 本次仅阅读本地源码差异、文档和已有运行记录，校验已有源码哈希；没有重新运行测试、构建、SITL或访问真实飞控。本报告不是实机部署或飞行放行单。

## 1. 一句话结论

项目已在原严格 V0 相对航点控制框架上增加可显式选择的 **V0-Research 科研策略、机载外层启动入口和遥测流请求节点**；已有离线回归、隔离SITL及DISARM只读集成证据，但仍存在未关闭的仿真遥测停顿问题，真实飞机部署与航迹验收尚未完成。

## 2. 当前代码状态与基本架构

- 当前分支：`feat/arducopter-v0-relative-waypoints`。
- 当前HEAD：`dcf3919d6b14d5531e4044c892bc4572e8875ffd`，最近提交时间为2026-10-28 16:20:30 +08:00，内容是GPS GUI飞行、故障注入及F3证据归档。
- 最新科研升级仍是未提交工作区增量。整理前有10个已跟踪文件修改，合计127行新增、23行删除，另有新增配置、节点、测试、部署文档和证据目录。因此，不能只凭HEAD复现当前结果，也不能假设网页端已能从远程仓库看到这些内容。
- `relative_mission_manager.py` 根据一次成功start冻结的P0/yaw0生成任务目标；`v0_nav_health.py` 是唯一的MAVROS位置目标发布出口。人工切出GUIDED优先，不自动抢回控制权或续飞。
- 真实部署设计链路是UWB经Pixhawk GPS接口进入ArduCopter EKF3，再由MAVROS输出；本次动态测试实际使用的是本机GPS仿真，不是声学/UWB真实链路。

## 3. 最新主要修改

### 3.1 严格模式与科研模式并存

新增策略由YAML字段显式选择，旧配置缺少新字段时仍采用严格默认值，不按文件名偷偷改变行为。

| 机制 | 严格 V0 / level | research |
|---|---|---|
| start前软件稳定悬停历史 | 必须满足 | 不要求该历史；基础健康与飞手确认悬停仍需满足 |
| 持续超速 | 中止 | 告警并记录速度，不因此中止 |
| 航点等待超时 | 中止 | 告警后继续等待，不跳点 |
| WPNAV速度/加速度性能范围 | 强制核验 | 只读记录及越界诊断，不据此拒绝 |

科研模式并未取消定位断流、GPS/EKF、原点变化、位置跳变、持续位置/速度冻结不一致、围栏、倾角、控制租约、节点失联和发布者竞争等联锁。固件及定位来源/接口核验仍保留。

同时新增累计告警、当前速度与速度峰值、飞控参数快照及性能诊断。修正了恰好0.35 rad倾角边界的浮点等价判定，未提高倾角上限。

### 3.2 三份任务配置

- `src/indoor_nav/config/mission_v0.yaml`：原严格基线，字节保持不变，右向方形。
- `src/indoor_nav/config/mission_v0_level.yaml`：新增严格门控、左向定高五点配置。
- `src/indoor_nav/config/mission_v0_research.yaml`：相同左向定高航点，显式启用上述四项科研策略。

两份新配置均为本地新建，并非从真实机载磁盘导出的历史原件。`z=0` 表示保持start时EKF高度，不代表离地高度为零。

### 3.3 机载入口与遥测请求

- 新增 `src/indoor_nav/launch/indoor_nav_real_v0.launch`，默认只读：`enable_control=false`、`dry_run=true`；默认选择严格level配置，research须显式选择。
- 可通过 `start_mavros` 决定是否启动MAVROS；已有采集MAVROS时必须复用，不能启动第二个。
- 新增 `src/indoor_nav/scripts/v0_request_streams.py`，通过已有 `/mavros/set_stream_rate` 服务请求EXTENDED_STATUS、POSITION、EXTRA1、EXTRA3、RAW_SENSORS五组遥测，配置频率分别为5/10/10/5/10 Hz。
- 该节点支持连接后请求、失败重试和重连重配，不新开MAVLink串口，不依赖pymavlink，不发送ARM、模式或控制目标。
- 测试工具新增 `--mission-file`，按案例保存实际模板及源码哈希。

## 4. 最近已有验证结果

以下来自2026-11-03已保存日志；本次仅复核，不是重新执行。

| 验证层级 | 已有结果 | 范围与限制 |
|---|---|---|
| `tests`全套 | 179项，OK | 包含新增回归及真实ROS消息/Launch解析测试 |
| `src/indoor_nav/test` | 9项，OK | 原核心回归，单独记录 |
| V0专项子集 | 101项，OK | 属于179项的一部分，不能重复累加 |
| 三份YAML离线smoke | PASS | 配置及坐标转换检查，不是飞行 |
| `catkin_make -j2` | 已有最终日志记录完成 | 不是本轮重新构建 |
| 隔离动态SITL | 11次尝试：10 PASS / 1 FAIL | research 10次、strict 1次；失败保留 |
| 外层Launch只读集成 | 最终level、research均PASS | DISARM状态，不调用ARM/起降/切模式 |

动态SITL覆盖单点、多点、显式abort、人工切模式、Manager正常退出、定位断流及Gate死亡。三批套件分别为6 PASS/1 FAIL、3 PASS、1 PASS；逐例记录stop成功及端口释放。

**重要限制：**动态测试会为具体case改写航点、使用 `fc_ekf_sitl_gps` 和条件化BRAKE策略，因此不等于两份生产五点YAML逐字原样完成了完整动态验收。生产导航节点不负责自动ARM/起降，这些动作来自隔离仿真测试驱动。

最终只读探针中，两份配置均拒绝DISARM状态下start，MAVROS位置目标发布者列表为空，五组流请求均有日志证据。每配置约20秒观测：level的pose/velocity/GPS/IMU约25 Hz，最大接收间隔约0.046 s；research约16.3–18.4 Hz，最大间隔约0.208 s。该环境已有MAVProxy流请求，不能把这些频率归功于新节点，也不能据此声称USB链路或请求频率已验收。

## 5. 必须保留的失败与未决问题

1. **动态SITL有一次真实FAIL。** `20261103T133514-manager_exit` 在预定Manager退出注入之前，因 `POSE_RX_TIMEOUT` 中止；观测pose age约0.508 s，实际最大接收间隔约0.974 s，超过0.5 s保护阈值。后续同类重试PASS，但没有覆盖原失败，也没有放宽断流阈值；该次停顿根因未定位。
2. **仿真链路稳定性尚未关闭。** 2026-10-28专项排查倾向于SITL↔Gazebo推进/数据交换瞬停；无GUI也复现，不能简单归因GUI或宣称CPU饱和。该历史证据不等于已证明11月3日失败具有同一根因。
3. **两次只读探针FAIL另行保留。** 首次过早读取MAVROS参数失败，后经身份核验定点清理残留；第二次因过早读取缓冲日志误报流请求缺失。最终修正探针后PASS，未为此修改生产遥测节点。
4. **离线测试也保留早期失败。** 禁用用户级Python包后旧测试缺少pymavlink，恢复普通测试环境后通过；另有Launch测试list/tuple期望错误，已修正测试。不能只保留最终绿色记录。
5. **坐标方向交接存在冲突。** 负右向代表左向，yaw0=0时对应正ENU y；历史交接中的某个负ENU y输出与之不一致。应核对机载原文件、起始航向和历史脚本，不能修改已验证坐标变换来迎合文字，也不能声称完全复现历史任务。

## 6. 证据身份与实机边界

- 本次重新计算原严格YAML SHA-256：`f33c125f3c3e9b0e89050b4a6ee5d4ec6050bce4a8ced49181aa989b16efa21b`，与保护记录一致。
- 本次对 `docs/v0/evidence/v0-research-20261103/final-files.sha256` 执行只读校验，全部通过；当前核心 `v0.py` SHA-256为 `d399ea87f1f9bec3e68fb93f73dbc352b4dd14ff84d54f13e91e63e1996cd5b2`。
- 已有部署包 `v0-research-deploy.tgz`、源码证据包 `validation-sources.tgz`、补丁和清单均在本次证据目录；包的存在不等于已同步到机载。
- 原验证记录称最终已清理本轮仿真与隔离端口。本次受限环境可见进程中未发现常见ROS/SITL/Gazebo进程名，但未进行全主机进程/网络审计，不据此作主机级保证。
- **尚无本次真实机载磁盘、真实USB/FCU、真实声学链路长期遥测、科研五点实飞、带动力RC接管、failsafe及采集联合验收证据。** 停止发布不等于物理刹停；条件化BRAKE也不是全场景保证。

## 7. 给网页端的讨论重点

请基于“本地软件回归与有限SITL已做、实机未验收”的状态理解本项目，不要将科研版解释为取消安全联锁，也不要把重试PASS写成全部成功。

后续若继续推进，优先讨论：机载实际文件与坐标方向核对；明确白名单、备份和差异后再进行人工批准的同步；先完成真实链路DISARM只读验证，再由人工决定有动力测试范围。若目标是稳定可重复的仿真验收，应单独排查SITL/Gazebo交换停顿，而不是放宽定位保护。本报告不执行这些后续动作。

### 推荐阅读材料（路径均相对工作区）

- `docs/v0/V0_RESEARCH_UPGRADE.md`：策略、职责和新增功能。
- `docs/v0/V0_RESEARCH_VALIDATION_20261103.md`：完整验证过程、数量、失败与未测项。
- `docs/v0/V0_RESEARCH_DEPLOYMENT.md`：部署交接与操作边界。
- `docs/v0/SITL_STALL_REVIEW_20261028.md`：仿真停顿排查及未决结论。
- `docs/v0/evidence/v0-research-20261103/sitl-index.json`：动态测试机器可读索引。
- `docs/v0/evidence/20261103T141057-readonly-streams/summary.json`：最终只读集成原始摘要。

网页端若无法访问本地文件，直接提供本报告；需要更深核对时，再附升级说明和验证记录。上述本地路径不是网页端已获得文件的证明。
