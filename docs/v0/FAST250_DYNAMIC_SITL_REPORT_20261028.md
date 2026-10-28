# Fast250接收时间改造后的真实 ArduCopter / Gazebo SITL 验证

日期：2026-10-28，Asia/Shanghai。分支：`feat/arducopter-v0-relative-waypoints`。

## 授权、版本与证据边界

用户本轮解除SITL暂停，允许隔离虚拟飞控ARM、起飞、航点、故障注入、降落及本地提交。**没有访问真实飞控设备、没有操作真实飞机、没有修改真实参数、没有push。** 定位为SITL内置GPS→EKF3→MAVROS，不使用ExternalNav，不开发Fake GPS模拟器。真机UWB留给另行授权的拆桨检查。

先完成Fast250代码冻结，再开始本轮第一次飞行：

| 提交 | 内容 |
|---|---|
| `af681a7cedf1e5dc532355aa5d261b11f1948437` | 本轮起点；既有安全门与暂停归档 |
| `ef0d2b2746b3199e176aa1e0d8f5c98a1664100b` | Fast250风格接收时间、配置、离线测试及原始失败bag对照；66项快速单测复核通过后提交 |
| `dc2599d964ae437014c8a5cda2d815ce3a7f7986` | 原SH的GUI开关、修复single/yaw用例被覆盖成四点、逐回调接收时间遥测 |
| `72d86aa2ddefb8603503a20edc8ba77c2ee0b3fb` | F3等待实际active录包文件、使用可处理的退出信号 |
| `255f495` | 容纳Gazebo GUI退出宽限；飞行门槛未改 |

Git对象标识按仓库实际为Git提交SHA；运行文件另用SHA256。每例 `inputs.json` 保存提交、实际源码/SH/SITL参数与mission YAML哈希；导航目录 `provenance.json` 再记录实际生产源码和加载YAML。新旧提交中的生产三文件与Fast250冻结版本相同，后续改动限测试/GUI/归档工具。工作区一直保留四份未跟踪的用户原始资料，不能宣称整个工作区干净。

## 启动流程与GUI

复用 `tools/v0_sitl_instance.sh`，而非默认加载ExternalNav的 `tools/sitl_flight_instance.sh`。保留原 `iris_ardupilot.world` 及Gazebo11物理模型，runtime副本保留原V0的1ms物理步长。只新增 `V0_SITL_GUI=true/false`：默认false保持兼容，本轮优先true，保留DISPLAY并把原来的 `gui:=false` 改为开关。没有为GUI取消隔离。

- Master：`http://127.0.0.1:11421`；Gazebo：`http://127.0.0.1:11455`。
- MAVROS：`udp://127.0.0.1:14672@`；MAVProxy只连本机SITL TCP5760；FDM UDP9002/9003。
- 每例独立 `.sitl_runtime/v0/<时间>-<PID>`，GPS实例不复制旧EEPROM；独占 `flight_claim.json` 禁止复用飞行。
- 固件本机源码tag为 `Copter-4.5.7`；实际二进制、defaults、world哈希在runtime `inputs.sha256`。
- 图形环境 `DISPLAY=:1`，X显示查询成功。已观测到gzclient、gzserver及映射的Gazebo窗口；证据 `evidence/fast250-dynamic-20261028/gui_process_audit.txt`。没有将只启动进程冒称飞行成功。
- 主机上两份旧MAVROS不属于本轮部署，保留不动；本轮“唯一MAVROS”限定于11421独立ROS图，不是宣称主机全局只有一份。

## 四航点准备超时与本轮修复

原故障完整因果证据见 `FOUR_WAYPOINT_ROOT_CAUSE.md` 与历史DataFlash解析。原实例连续飞行后home=583.86m、origin=583.99m，`1m ABOVE_HOME`换算约0.87m ABOVE_ORIGIN；控制目标约0.87m、实际约0.86m，Gazebo真值最高0.83423m，未达到驱动0.85m世界高度门槛。任务仍IDLE，因此不能归为航点坐标算法失败。不要把home、EKF原点、Gazebo世界零点当作相同。

本轮每例冷启动、独立EEPROM，保持以下原判据：起飞命令1.0m；真值Z>0.85m；速度三轴范数<0.06m/s；35秒准备超时。**没有降低高度/速度门槛，也没有延长准备时间。**

`20261028T142721-multi`：四航点PASS，准备真值Z=0.996074m、速度约(0.01355,-0.01436,0.02394)m/s，范数约0.031m/s。完成后LAND/DISARM，runner确认端口释放。原20261026 FAIL仍为FAIL；本轮PASS证明独立冷启动在这些条件下通过，不证明复用实例问题或高度偏差的所有物理来源已根治。

第二次独立冷启动 `20261028T150850-multi` 同样四点PASS。两次到达时Gazebo水平误差最大分别0.04985/0.04547m，Z误差最大0.00683/0.00741m；实际FC目标发布约9.992Hz，最大bag接收间隔约0.10034/0.10095s。保留各点原始记录，不把到达瞬间误差当全航段最大误差。

另发现测试驱动独立缺陷：为single/yaw设置单点后，下一段distance判断的else又覆盖为四点。已抽取 `test_waypoints()`、修复分支并添加单测。本轮single确为1点，multi确为4点；不能只凭历史case名认定历史飞行是单点，应查原YAML与summary。

## 通信与Fast250方案的动态边界

参考固定版本和源码对应关系见 `FAST250_RECEIPT_TIME_REVIEW_20261028.md`；该文件的“暂停”描述是此前离线阶段快照，不回写历史。生产节点继续用ROS回调接收年龄和monotonic停流上界；原始header只做诊断，EKF、坐标系、跳点、原点、人工接管和唯一Gate保护仍保留。

配置仍为pose/velocity/command 0.5秒、state显式3秒。state实测约1Hz，不能机械套0.5秒。`receptions.jsonl`保存测试驱动每次回调的ROS/monotonic接收时刻、原始header、frame、间隔；生产Gate自身时间与诊断在watchdog话题/包中，不能拿驱动时间代替生产回调时间。

明确保留的新失败：

- `20261028T143245-yaw`：在HOLD_INITIAL实际停流。驱动pose最大回调间隔0.547140s，bag pose/velocity间隔约0.54715s，原始MAVLink也约0.54066s；manager/gate状态话题仍约0.1秒持续更新，raw pose无回退。manager接收age约0.515s触发 `POSE_RX_TIMEOUT`。这不是header偏移误判，也不是单个manager回调锁能解释的现象；链路/模拟器停顿根因尚未定位。
- `20261028T143753-mode_exit`：同样在真正模式切出注入前因接收超时中止，不能把此例标为GUIDED退出验收成功。

Fast250方案不应掩盖真实停流；本轮不以放宽0.5秒、改写消息头或自动续飞让这些FAIL变成PASS。后续有界冷启动补测需单独保留结果，不能覆盖以上记录。

同阈值、同GUI配置下的一次有界补测：`20261028T150340-yaw` 单点PASS，`20261028T150619-mode_exit` 切出GUIDED并人工切回后仍锁存、start拒绝，PASS。bag原点四元数核验：普通单点初始ENU yaw约1.53294rad，改变机头后的yaw用例捕获约-0.00590rad；均按冻结任务轴执行。没有在本轮飞行中额外注入header倒退；此项仍由此前原始失败bag对照与离线测试覆盖，不能冒称已主动飞行注入。

## 本轮矩阵（包含失败，不挑选最佳结果）

目录均位于 `docs/v0/evidence/`，每例有summary、inputs、mission、telemetry、events与cleanup；大bag保留本地。

| 案例/证据目录 | 结果 | 范围 |
|---|---|---|
| `20261028T135218-single` | 飞行PASS / suite清理FAIL | 真正1点；已DISARM；初版GUI清理等待不足 |
| `20261028T140034-single` | 飞行PASS / suite清理FAIL | 第二次1点；已DISARM；退出信号调整仍未覆盖GUI宽限 |
| `20261028T142721-multi` | PASS | 全新实例4点、原悬停门槛、独立清理 |
| `20261028T143012-single` | PASS | 全新实例1点 |
| `20261028T143245-yaw` | FAIL | HOLD_INITIAL真实接收停流，未完成航点 |
| `20261028T143517-altitude` | PASS | 上0.1m、回启动Z；2点 |
| `20261028T143753-mode_exit` | FAIL | 模式故障注入前真实停流；不计接管PASS |
| `20261028T144020-abort` | PASS | 任务锁存、健康Gate请求BRAKE、观测减速 |
| `20261028T144252-manager_kill` | PASS | 真正SIGKILL、心跳失效、健康Gate请求BRAKE |
| `20261028T144525-gate_kill` | 检测PASS / 刹停未保证 | Gate真正SIGKILL；FC继续旧目标 |
| `20261028T144757-pose_loss` | 检测PASS / 刹停未保证 | 两导航节点的pose输入停止；未破坏FC GPS |
| `20261028T145026-mavros_loss` | PASS（失联/恢复锁存） | 真正终止本轮MAVROS、替换实例重连；未自动续飞；不宣称失联自动刹停 |
| `20261028T145308-concurrent` | PASS | 并行start/abort/stop；不接受不当start/stop，不自动恢复 |
| `20261028T145540-f3` | PASS（模拟负载） | 音频/手动recorder退出不停止导航；manager退出录包继续 |
| `20261028T145816-f3_auto_bag` | PASS（模拟负载+真实本地recorder源码） | ARM起包、DISARM关闭；物理传感器未验收 |
| `20261028T150340-yaw` | PASS（独立补测） | 改变初始机头后1点；不覆盖先前FAIL |
| `20261028T150619-mode_exit` | PASS（独立补测） | ALT_HOLD回读、切回GUIDED仍不恢复 |
| `20261028T150850-multi` | PASS（重复性） | 第二次独立冷启动4点 |
| `20261028T151140-dry_run` | PASS | 全程DISARM，无V0控制publisher |

共19份独立summary：17 PASS、2 FAIL；另有两次早期suite清理FAIL，不能用其内部飞行PASS把suite改成PASS。13例主批次为11 PASS/2 FAIL，四例补测批次全部PASS。离线全量152项单测PASS（含74项V0，不重复计数）；包内另9项PASS；catkin目标包构建PASS，AST/API/launch及旧代码保护检查PASS。日志在 `evidence/fast250-dynamic-20261028/`。

最新通过不代表总体已没有间歇性链路停顿；本轮仅证明新接收机制不会把原始header回退直接当断流，真正超过门槛的停流仍正确中止。

## 安全判据与剩余风险

将“任务锁存”“Gate停止输出”“飞控模式回读BRAKE”“实际减速/停稳”分开报告。生产默认 `manual_takeover`；本轮驱动显式配置 `brake_if_position_healthy`，只在独立位置/EKF健康且故障白名单满足时由唯一Gate请求BRAKE。不能把仿真配置直接视为真机放行配置。

已观测到：

- 正常abort和manager SIGKILL：Gate撤销、BRAKE回读、FCU测得水平速度低于0.04m/s；Gazebo真值仍有约0.05m/s残余，不能把单次FC速度门槛称为物理瞬时完全停稳。完整减速过程在telemetry。
- Gate SIGKILL：manager锁存但FCU仍ARM/GUIDED，观测真值水平速度约0.218m/s。
- 消费端pose断流：manager/gate锁存，不盲目BRAKE；FCU仍ARM/GUIDED，观测真值水平速度约0.220m/s。该用例阻断的是导航pose输入，不是SITL GPS/EKF真实失效。

后两项的检测测试PASS不是“故障后安全刹停PASS”。SITL参数 `FS_GCS_ENABLE=0`，也不能宣称FC侧通信丢失会自动制动。Gate或MAVROS死亡时，RC人工接管与另行实物验证的飞控failsafe仍是最后防线；禁止室内默认RTL，禁止未知定位时强行Loiter/BRAKE。

实际MAVROS SIGKILL用例：约0.673秒观察到锁存；恢复使用新的本轮MAVROS进程，身份校验等待SIM参数真正可读后才允许清理LAND，最后DISARM并停止SITL，再关替换MAVROS。失联期间模式/速度可能是最后一份MAVROS样本，不作为新鲜FC确认；Gazebo真值当时仍约0.217m/s，不能宣称已经停稳。原始失联及恢复日志都保留。

本机 `ArduCopter/mode_guided.cpp` 的 `get_timeout_ms()`注释明确其用于速度/加速度/角度等控制；位置控制中停止更新不清除既有位置目标。`posvelaccel_control_run()`的超时分支将速度/加速度归零而继续位置闭环。因此不能把停发位置目标解释成取消飞控位置意图。此结论还有上述Gate失效动态观测支持，不仅凭日志“已中止”。

## 速度解释

保留 `WPNAV_SPEED=20cm/s`、UP/DN=20cm/s、ACCEL/ACCEL_Z=50cm/s²。0.20m/s是期望巡航，不是瞬时硬上限；4.5.7 Guided使用位置误差修正和加减速控制，实测应分别记录启动、航段、制动与切换。每例 `bag_analysis.json` 含FC速度峰值、超0.20/0.30m/s的采样覆盖持续时间、按航点/状态分组和真值误差；采样统计不声称连续时间硬保证。没有放宽物理围栏或持续超速保护。

两次四点GUIDED阶段FC水平峰值0.26008/0.25872m/s，超过0.20m/s的采样累计约2.883/2.840秒，最长连续覆盖约0.882/0.878秒；没有观测到超过0.30m/s。单点峰值约0.21350m/s。manager-kill例整个“最后任务状态”窗口另有0.34772m/s峰值，按FC mode分组属于后续测试驱动LAND清理段，超过0.30m/s采样覆盖约0.524秒，不能隐去，也不能误称GUIDED巡航峰值。mode回读有心跳更新延迟，分段不是飞控内部精确时刻。manager死亡后最后任务status会持续缓存，因此工具新增mode分组及此告警，不只按缓存status归因。

## 声学采集与导航共享MAVROS

没有修改采集工作区或物理音频协议。F3使用 `tools/v0_mock_acquisition.py` 的10Hz合成 `UInt8MultiArray`：XVF大小19208字节、六路48kHz/24bit等大小motor数据86408字节、振动72字节（各含序号前缀）。仅证明通信大小/时序和进程独立性，绝不是实际PCM/ADC/USB/ESC质量验收。

此前F3的 `after_recorder.bag.active` 直接stat发生在recorder异步启动尚未创建文件时，原失败不改写。现在先确认recorder进程未退出、实际 `after_recorder*.bag.active` 出现且有内容，再比较manager退出前后增长；最终还检查关闭后的bag可读，不把最终 `.bag` 误当活动录包证据。添加了不存在active文件/只有已关闭bag/recorder已退出的单测。

- F3：manager退出后手动bag增长2,990,059字节；F3_auto：手动bag增长3,096,555字节，自动bag增长3,305,472字节。
- 两例均验证音频进程退出和原recorder退出后导航继续，manager退出后音频/recorder继续；ROS图共用同一Master及唯一MAVROS，只有Gate发布FC目标。
- FC目标bag最大间隔约0.10035/0.10042秒；驱动pose回调最大间隔约0.08858/0.05882秒；没有靠扩大超时容忍满负载。
- 各已录制epoch内序号缺失0；自动bag跨音频重启观察到1次序号reset，符合主动kill/restart，不把整个主动停机间隔称为“音频连续”。录包订阅前/关闭后的数据不在证明范围。
- 只读调用本地 `onboard_ros_sensor/ros_sensor/src/rs485_driver_sound/scripts/auto_bag_recorder.py`，独立输出本轮evidence；ARM起包、DISARM正常关闭，关闭包首个state为ARM/GUIDED、末个state为DISARM/LAND，`rosbag check`无需迁移且只读扫描可读。
- `system_cpu_percent_max`约96.2/98.5只是测试采样峰值，不代表全程持续该占用率。没有真实IM72等硬件及机载最新版联合证据，**真实硬件R4仍BLOCKED**。

如将来给机载录包追加导航话题，只在人工确认当前最新版launch后追加原列表，不替换它。示意差异（占位符不是可直接覆盖机载的补丁）：

```diff
- <param name="topics" value="原有完整话题列表"/>
+ <param name="topics" value="原有完整话题列表 /v0_nav/status /v0_nav/event /v0_nav/origin_pose /v0_nav/target_pose /v0_nav/heartbeat /v0_nav/watchdog"/>
```

## 证据索引、失败保留与结束状态

- 有效新索引：`evidence/fast250-dynamic-archive-20261028-verified/run_matrix.json`、`historical_index.json`、`manifest.json`；逐例原始状态与源码/YAML身份一并列出。只新增归档，不重写历史全局manifest或旧报告。归档工具首次处理相对support路径时报 `ValueError: ... does not start with ...`，半成品目录 `fast250-dynamic-archive-20261028/`保留；修复路径归一化后，`...-final/`迭代又将已完成但含FAIL的主批次误标INCOMPLETE，未误标PASS。已修复并补充路径及批次分类单测，最终只以 `...-verified/`为准，前两次派生迭代保留，不修改任何飞行结果。
- `runtime_logs/`保存本轮独立runtime的关键日志文本副本及输入哈希；本地原bag、DataFlash BIN、tlog、EEPROM仍在原路径，manifest包含大小/SHA256，未纳入Git。需要迁移证据时应另行复制这些本地大文件，单靠Git不能复原全部二进制。
- 两次旧清理失败的suite目录、两次真实停流FAIL及所有此前失败保留。Gazebo统计查询曾两次遇到实例切换间隙，返回“无运行实例”，输出保留但不用于证明实时率；有效GUI证据为窗口/进程审计和运行日志。
- GUI退出时roslaunch等15秒后升级终止gzclient，原stop的约18秒总等待与其竞争；改为45秒主等待、外层70秒期限后批次清理成功。只改变DISARM后清理期限，不改变起飞/悬停/健康/航点条件。
- 最终主机检查 `evidence/fast250-dynamic-20261028/final_host_audit.txt`：无本轮Master/MAVROS/Gazebo/SITL/导航/录包残留，11421/11455/5760/9002/9003/14672未占用；两份旧MAVROS PID2391064/3263919仍在，未操作它们。每例cleanup均确认connected、DISARM及测试子进程退出。
- 交付暂存区检查：代码、测试、说明文档的 `git diff --cached --check`通过；全量检查对原始MAVROS/CMake/ss日志报告尾随空白。为保持证据字节与manifest哈希一致，保留日志原样，不把该全量检查写成无告警PASS，也不为通过格式检查篡改原始日志。

**下一步不是直接带桨自动飞行。** 可在另行实物授权后按SOP开展拆桨R0接口/方向/EKF/RC调查；间歇性真停流的链路/模拟器根因、Gate/MAVROS死亡的飞控侧应急能力、真实UWB质量与满载采集仍须关闭风险。R1/R2必须现场负责人及飞手人工签核，不用本轮PASS替代物理防护。

## 复现命令与人工停止

所有下列SITL命令仅在本机已验证的隔离环境使用，不能在真实飞控Master上运行：

```bash
cd /home/liqidun/github/indoor_nav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11421 ROS_IP=127.0.0.1
export V0_SITL_GUI=true
python3 tools/v0_sitl_suite.py --sitl --execute --cases multi single yaw altitude mode_exit
python3 tools/v0_sitl_suite.py --sitl --execute --cases abort manager_kill gate_kill pose_loss mavros_loss concurrent f3 f3_auto_bag
```

runner调用原SH，逐例新runtime，身份核验后才ARM，结束LAND/DISARM再清理；有清理失败即停止下一飞行。单独启动时：

```bash
V0_SITL_GUI=true ./tools/v0_sitl_instance.sh --sitl
# 另一同环境shell；只在当前实例尚未执行过飞行时运行：
python3 tools/v0_sitl_test.py --sitl --execute --case multi
# 已connected且DISARM才允许停止：
python3 tools/v0_sitl_stop.py --sitl
```

`rosservice call /v0_nav/abort '{}'`只表示撤销任务；需另观测mode/速度，必要时使用已核验SITL测试驱动的LAND清理，不将本报告的仿真ARM/起降方式带到机载。

离线及只读：

```bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
python3 tools/test_v0_relative_waypoints.py
roslaunch indoor_nav indoor_nav_v0.launch enable_control:=false dry_run:=true
python3 tools/test_v0_relative_waypoints.py --ros-smoke
rosbag info --yaml /absolute/path/to/closed_flight.bag
rosbag check /absolute/path/to/closed_flight.bag
```

F0保持旧配置独立；本轮跑旧/新单测和构建，没有重新ARM跑历史ExternalNav。F2按用户决定取消，无新的模拟命令。真机后续清单继续以 `REAL_AIRCRAFT_SOP.md` 为基础：拆桨只读、唯一MAVROS、UWB方向/EKF、RC模式接管、速度单位、P0/yaw、实体净空、人工悬停dry_run、单个0.2–0.3m点、再小方形与音频满负载。**本报告不是带桨自动航点放行证明。**
