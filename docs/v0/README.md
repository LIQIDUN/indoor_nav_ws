# ArduCopter V0 相对航点

## 2026-11-03 V0-Research 与机载入口

严格 `mission_v0.yaml` 原样保留。新增 `mission_v0_level.yaml` 是严格门控的左向定高方形参考；
`mission_v0_research.yaml` 使用相同左向航点，但关闭软件稳定悬停历史要求、将超速与航点超时降为去重告警、只记录 WPNAV 性能参数。
定位、姿态、围栏、控制租约、飞手接管及固件/接口联锁仍保留。不得把 level 与 research 混为一谈。
原严格样例是正右向方形，新文件是负右向（左向）方形；并非同一水平任务。

详见 `V0_RESEARCH_UPGRADE.md`、`V0_RESEARCH_DEPLOYMENT.md` 与 `V0_RESEARCH_VALIDATION_20261103.md`。
以下历史测试命令和报告不代表本次科研版已完成全部动态或真实飞机验收。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# USB 一键入口默认只读，默认严格 level 配置；不 ARM、不切模式、不起飞。
# 若采集系统已启动 MAVROS，必须追加 start_mavros:=false。
roslaunch indoor_nav indoor_nav_real_v0.launch enable_control:=false dry_run:=true
# 选择科研版仍需显式指定任务；已有 MAVROS 的示例：
roslaunch indoor_nav indoor_nav_real_v0.launch start_mavros:=false \
  mission_file:="$(rospack find indoor_nav)/config/mission_v0_research.yaml" \
  enable_control:=false dry_run:=true
```

新增入口可启动唯一 USB MAVROS，并在连接/重连后请求 EXTENDED_STATUS/POSITION/EXTRA1/EXTRA3/RAW_SENSORS。
请求服务无 success 字段；调用完成不证明实际频率达标，需测 `rostopic hz`。
导航入口 `indoor_nav_v0.launch` 仍不启动 MAVROS，`mission_file/evidence_dir/pose_topic` 透传至两个节点。

**默认只读；2026-10-28已另行获准恢复隔离SITL动态测试，未获任何真机操作授权。**

2026-10-26范围修订见 `DEVELOPMENT_PLAN_REVISION_20261026.md`：用户确认UWB经飞控GPS接口提供定位；F2专项模拟及协议调查已取消。10月27日暂停快照见 `PAUSE_REPORT_20261027.md`。10月28日Fast250离线改造见 `FAST250_RECEIPT_TIME_REVIEW_20261028.md`，恢复后的GUI飞行、失败与安全边界见 `FAST250_DYNAMIC_SITL_REPORT_20261028.md`。历史报告按当时状态保留，没有操作真机。

这是 `indoor_nav` 包的增量模式，不取代旧 ExternalNav。manager只发布任务意图 `/v0_nav/target_pose`；现有 `v0_nav_health.py` 作为gate，是唯一 `/mavros/setpoint_position/local` 发布者及可选BRAKE模式请求出口。gate独立观察定位、EKF、任务租约、时间和ROS图，失效后锁存；不是第二个无协调控制发布者。生产节点不 ARM/起降、不写FC参数、不管理rosbag/音频进程。

默认 `control.exit_action: manual_takeover` 不自动切模式。显式 `brake_if_position_healthy` 仅在任务曾获gate所有权、仍ARM/GUIDED、独立定位/GPS/EKF及4.5.7参数快照新鲜时单次请求BRAKE；定位失效、MAVROS断连、人工切模式、gate自身退出不保证刹停。测试程序显式使用此策略；真机是否启用须R0人工核查后另行批准。SetMode返回不等于实际停稳。

控制节点在DISARM时启动，让gate先观察IDLE/DISARM；其后由飞手人工起飞/悬停再start。gate不支持半空重启自动接管，断链或中止后必须落地、显式reset并重新授权。独立健康检查仍消费同一个FC EKF，不是第二套定位源。

## 构建、回归、只读检查

```bash
cd ~/github/indoor_nav_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
python3 tools/test_v0_relative_waypoints.py
# F0 原有 offline smoke；未启动飞行、未声称重飞历史 run_06
./tools/indoor_nav_sitl_smoke_test.sh
```

硬件无关的离线 smoke 解析完整 YAML 并打印 yaw=0 目标。ROS dry_run 需已有单一 Master/MAVROS；启动 V0 自身不会 include MAVROS 或旧节点：

```bash
# 实机只读：此 shell 使用与机载最新采集系统一致的 ROS_MASTER_URI
# 先在采集 shell source 机载实际工作区，并启动最新已验证采集入口
# 不要将开发机旧 ros_sensor launch 覆盖过去
unset ROS_HOSTNAME
source /opt/ros/noetic/setup.bash
source ~/github/indoor_nav_ws/devel/setup.bash
rosparam get /use_sim_time                 # 实机必须不是 true
roslaunch indoor_nav indoor_nav_v0.launch enable_control:=false dry_run:=true
# 另一个共享相同 Master 的 shell：
python3 tools/test_v0_relative_waypoints.py --ros-smoke
```

启用控制需同时 `enable_control:=true dry_run:=false`；**两把开关同时约束manager和gate**。此命令不等于准许实飞；先完成SOP对应门槛。默认 `navigation.source: fc_ekf_gps`，GPS/EKF反馈与真实来源一致，不要求自定义adapter；`require_upstream_health: true` 是可选增强项，未启用时不订阅健康值作为强制前提，不能常量true伪造验证。只读时gate不创建MAVROS控制publisher或请求模式；target_pose是意图/预览，不是FC执行回执。

## F1 隔离仿真（自动 ARM/起飞只在专用测试程序中）

推荐使用冷启动批测工具，每例新建runtime/EEPROM、核验路由、落地后安全停止；一次runtime只允许一个飞行案例。原高度>0.85m、速度范数<0.06m/s、35s准备门槛未放宽：

```bash
source /opt/ros/noetic/setup.bash
source ~/github/indoor_nav_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11421 ROS_IP=127.0.0.1
python3 tools/v0_sitl_suite.py --sitl --execute --cases single multi yaw altitude
python3 tools/v0_sitl_suite.py --sitl --execute --cases abort manager_kill manager_exit manager_pause pose_loss mavros_loss mode_exit gate_pause gate_kill concurrent publisher_conflict
python3 tools/v0_sitl_suite.py --sitl --execute --cases f3 f3_auto_bag distance_short distance_long finish_stop
```

`f3_auto_bag` 只读运行本机 `onboard_ros_sensor` 的自动recorder源码并把输出定向到本轮证据，传感器仍为mock；没有启动其硬件launch。手工启动方式如下，**同一实例不要连续运行两个飞行案例**：

关闭其它占 TCP5760、UDP9002/9003 的仿真；不要杀不属于本次任务的进程。第一个终端：

```bash
./tools/v0_sitl_instance.sh --sitl
# 不 ARM；保持前台。结束前确认仿真已降落 DISARM，再 Ctrl-C。
```

第二个终端（每次测试自动生成新证据目录、YAML 和独立导航实例）：

```bash
source /opt/ros/noetic/setup.bash; source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11421 ROS_IP=127.0.0.1
python3 tools/v0_sitl_test.py --sitl --execute --case single
python3 tools/v0_sitl_test.py --sitl --execute --case multi
python3 tools/v0_sitl_test.py --sitl --execute --case f3
python3 tools/v0_sitl_test.py --sitl --execute --case link_fault
# 测试已降落DISARM后，验证PID/实例后关闭本轮仿真：
python3 tools/v0_sitl_stop.py --sitl
```

建议**每个case使用一次冷启动仿真**：一个case完成并DISARM后执行stop，再由第一终端重启。连续复用同一EEPROM/运行实例可能保留高度/起飞状态；本轮有一次第二次起飞真值仅约0.83m，未达测试准备门槛而超时，任务未start。不要降低门槛掩盖它；保留失败，冷启动后再测。

测试必须核验运行目录、SITL 可执行文件/PID、MAVProxy 唯一路由和 loopback socket、MAVROS arming 服务归属和 SIM 参数；不匹配直接拒绝。`multi` 起飞后先用仿真专用 yaw 命令改变初始航向，再冻结任务轴，含正 Z 点。F1 使用 **SITL 默认 GPS→EKF3**，即修订后的仿真定位路径。F2专项及底层协议调查已取消，无需开发等价注入；真实UWB定位质量/方向/EKF状态在R0核查。以下命令仅供用户明确恢复后使用。

F0动态回归另起全新runtime，不覆盖旧EEPROM，且不得与F1同时运行：

```bash
./tools/v0_sitl_instance.sh --sitl --f0-externalnav
# 另一shell同样 export ROS_MASTER_URI=http://127.0.0.1:11421 ROS_IP=127.0.0.1
python3 tools/set_ekf_origin_once.py --lat=-35.363261 --lon=149.165230 --alt=584
rosservice call /mavros/cmd/command '{broadcast: false, command: 512, confirmation: 0, param1: 49.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}'
rostopic echo -n 1 /mavros/global_position/gp_origin
rosservice call /indoor_nav/calibrate '{}'
./tools/indoor_nav_sitl_smoke_test.sh
# 不ACK、不ARM、不起飞；这只是最小旧链路回归，不是重飞run_06
# 此固定经纬度原点设置仅用于F0，绝不复制到真实飞机；不是GPS注入
```

## 配置契约

默认 `src/indoor_nav/config/mission_v0.yaml`，兼容用户样例。严格拒绝重复 YAML key、未知字段、缺字段、重复 ID、NaN/Inf、布尔代替数字、空航点、负时间、极端值与超界点。所有点为米、相对唯一启动 P0/yaw0；`(x,y,z)` 为前/右/上，不累计、不随当前机头旋转。`z=0` 保持启动 EKF 高度，不是 AGL。逐点 yaw 不支持，误填会拒绝；默认始终保持 yaw0。

四元数归一化、拒绝零范数；角度跨 ±π 正确处理。目标 ENU，右向先视为 FLU 负 Y，由 MAVROS 做 NED 转换，不在业务层重复换轴。

全部安全缺省值见 `indoor_nav/v0.py:SAFETY_DEFAULTS`，可在 `safety` 内显式覆盖：state/GPS/EKF新鲜度、速度/pose新鲜度、跳点、持续超速、悬停窗口/范围/角度、initial hold、ROS/墙钟偏差、周期/watchdog期限、冻结一致性。限制不是物理防护。软件 fence 检查所有目标和飞行当前位置。小于检测门槛的原点变化仍可能漏检。

位姿、速度、state与任务指令的新配置集中在 `msg_timeout`，单位秒，代码缺省均0.5。样例pose/velocity/command为0.5，state显式3.0以兼容约1Hz心跳；应按实测频率选值，不要把state的0.5秒缺省直接用于1Hz输入。legacy `safety.pose_timeout_s / velocity_timeout_s / state_timeout_s / command_timeout_s` 仍可读；两处同时配置且值不同会拒绝加载，不静默覆盖。旧YAML的0.35秒pose门槛仍是0.35。

Fast250式新鲜度以回调的ROS接收时间计算，不把原始header的偏移、重复或单次回退作为接收拒绝门槛；保留monotonic接收间隔上界，避免ROS暂停掩盖真实断流。原始stamp保存在状态的 `header_stamps_s`，诊断在 `stamp_diagnostics`，接收时刻在 `rx_ros_s`；这些是最新快照，逐消息分析应保留原始MAVROS topic录包。pose frame仍只接受已审计的 `map`，velocity仍兼容已审计插件的 `map/base_link` 标签，不改变ENU数值。

`POSE_FRAME_INVALID` 表示坐标系错误，`POSE_RX_TIMEOUT` 表示消息接收超时；`RAW_STAMP_REWIND / OFFSET / REPEAT / INVALID` 是原始消息头诊断，不自行重建时间同步。ROS时间未初始化禁止启动；归零/倒退清空旧接收状态，活动任务锁存中止，不重捕获原点续飞。断链清空旧pose/velocity/GPS/EKF，重新连接仍须新消息及人工授权。回调持续不能证明真实物理定位正确，GPS/EKF、跳点、原点和冻结一致性检查保留。

必须先观测 `/mavros/global_position/gp_origin`，任何后续原点变化锁存中止（包括pose未跳变时）。该通知不覆盖所有EKF内部reset；仍需上游真实reset证据。Copter4.5.7实际通过既有 `/mavlink/from` 的 `mavros_msgs/Mavlink`、msg193 EKF_STATUS_REPORT提供健康；生产仅接收CRC有效、sysid/compid=1/1的固定消息，使用标准库struct解码并检查flags/ratios，不引入pymavlink或新连接。通用msg230的estimator_status可能完全不出数，不能臆造健康值。

**严格配置**的 `speed_xy_mps` 是对人工预设 FCU 限速的上界要求，不是插值速度控制：Copter4.5.7 必须 `20 <= WPNAV_SPEED <= 100*speed_xy_mps`。UP/DN默认≤20cm/s，ACCEL默认≤100cm/s²；持续超速中止。科研配置 `speed_enforcement: record_fcu_limits` 不强制这些性能范围，保留只读快照和越出严格基线的诊断；`overspeed_action: warn` 不因持续超速本身撤销任务。两种配置仍要求 GUID_OPTIONS=0、EKF3/GPS来源匹配、固件4.5.7及新鲜接口快照。任何模式都不自动改FC参数。

策略缺省兼容旧版：`start_policy.require_stable_hover: true`、`motion.overspeed_action: abort`、`arrival.timeout_action: abort`。科研版分别使用 false / warn / warn_and_wait。超时不会跳点；必须满足 XY/Z 连续到达和 hold_s 才推进。`/v0_nav/status` 增加 warnings、speed、speed_peaks、fcu_parameters、performance_diagnostics；峰值覆盖节点启动/显式reset后的有效速度回调，不是硬件极限。超速每任务最多一条告警、超时每航点最多一条；当前速度与历史告警应分开读。

实测WPNAV_SPEED=20cm/s时位置修正仍有约0.266m/s短时峰值，因此**0.20不是实际速度的严格瞬时上限**；本轮验证的是FC参数生效及0.30m/s实测监测门槛。若实验要求任何时刻≤0.20m/s，当前V0不满足，不能仅修改YAML宣称实现。

## 服务与状态

```bash
rosservice call /v0_nav/start '{}'  # 飞手人工悬停、GUIDED、健康与限速检查后显式授权
rosservice call /v0_nav/abort '{}'  # 锁存、停止输出；不保证取消飞控最后目标！飞手接管
rosservice call /v0_nav/stop '{}'   # 仅 FINISH_HOLD→STOPPED；gate依exit_action处理
rosservice call /v0_nav/reset '{}'  # 仅连接正常、DISARM、非活动状态；仍须再次显式 start
```

状态：`IDLE→WAIT_FOR_READY→CAPTURE_ORIGIN→HOLD_INITIAL→EXEC_WAYPOINT→DWELL→FINISH_HOLD`；完成持续发最后目标直至人工切走模式/stop。异常 `ABORT_LATCHED`，无自动续飞，无自动抢回 GUIDED。初始先发布冻结当前点并 hold，只有飞手预先进入 GUIDED且持续回读正确才推进；不实现程序切 GUIDED。双 start 拒绝。到达须 XY/Z 同时连续满足 stable_time_s，DWELL 离开容差则重新稳定，超时取消全部后续点。

定位坏时绝不把“当前位置保持”称为安全；此时停发且不盲目切BRAKE/Loiter，依赖RC/FCU预验证应急。定位健康的任务程序故障可在显式策略下由唯一gate单次请求BRAKE，须观察mode和速度确认效果。没有默认RTL/LAND；gate进程自身死亡或完全断连不能由伴随计算机保证处置。

## ROS 接口与证据

| Topic | 类型 | 内容 |
|---|---|---|
| `/v0_nav/status` | std_msgs/String | 10Hz小型JSON：state/mission_id/wp/reason/health/gate/control_enabled/pose_age/最大publish间隔/证据路径 |
| `/v0_nav/event` | std_msgs/String | 稀疏JSON：seq/mission_id/wall_s/ros_s/mono_delta_s/事件与状态；开始含固件与参数快照 |
| `/v0_nav/origin_pose` | geometry_msgs/PoseStamped | latched，冻结 P0/yaw0，map |
| `/v0_nav/target_pose` | geometry_msgs/PoseStamped | 预览目标，dry_run也可有，不代表飞控回执 |
| `/v0_nav/heartbeat` | std_msgs/Header | 10Hz活性，非控制 |
| `/v0_nav/watchdog` | std_msgs/String | healthy/reason/owner/revoked/exit_result/mode/实际目标最大发布间隔；gate状态，不是刹停认证 |
| `/v0_nav/upstream_healthy` | std_msgs/Bool | 仅require_upstream_health=true时强制检查；默认普通GPS不要求此自定义信号 |

订阅State、map PoseStamped、指定velocity_local TwistStamped、raw/fix NavSatFix、gp_origin及既有MAVROS的msg193转发。manager读取ParamGet/VehicleInfoGet并记录固件/参数；gate独立检查关键导航反馈。GPS fix本身不能证明真实UWB链所有质量属性，须R0现场检查；不再开发专项adapter。冻结检测不能识别所有同源共同冻结。

每次实例创建独立证据目录：原始YAML、绝对路径和SHA256、Git/dirty、源类型、固件版本/FC参数、wall/ROS/monotonic事件。默认 `~/.ros/v0_nav`，可 `evidence_dir:=...`。任务节点只持久化稀疏事件，遥测由已有采集系统录制；测试驱动另存实际位姿/真值/目标/速度/模式/echo/bag。ROS接收时间用于消息新鲜度与录包对齐，monotonic用于任务期限、周期和接收停流兜底；主循环用墙钟等待，暂停 `/clock` 不会让 watchdog 线程静止，但不等于所有时钟异常都可被检出。

## 采集并行与录包

不改采集文件。现场确认最新 recorder 的 `topics` 参数后，**只向现有列表追加**：

```diff
-<param name="topics" value="原有完整话题列表"/>
+<param name="topics" value="原有完整话题列表 /v0_nav/status /v0_nav/event /v0_nav/origin_pose /v0_nav/target_pose /v0_nav/heartbeat"/>
```

这是人工审阅的示意最小 diff，不是可直接 apply 的机载补丁（本机没有该最新版）。保留 XVF/IM72/ESC/振动所有原话题，不能把示意文字当列表。录包包含真实 `velocity_local`（附件旧 `/velocity` 不一定存在）。导航不触碰 recorder；真实自动 ARM/DISARM 录包仍需 R0/R4。

```bash
rosnode list
rostopic info /mavros/setpoint_position/local
rostopic info /mavros/setpoint_raw/local
rostopic type /mavros/local_position/velocity_local
lsof /dev/ttyACM0                      # 实机：唯一串口占用者应为现有 MAVROS
rosbag info --yaml <本轮证据目录>/flight.bag
rosbag check <本轮证据目录>/flight.bag
python3 tools/v0_analyze_bag.py <本轮证据目录>
# 仅在没有实机/导航订阅的隔离离线 Master 回放；不要向实机 Master rosbag play 控制话题
```

细分实测与未完成项见 `SITL_REPORT.md`、安全责任见 `SAFETY_AND_LIMITATIONS.md`。

## 变更清单

| 文件（相对工作区） | 原因 |
|---|---|
| `src/indoor_nav/src/indoor_nav/v0.py` | 独立schema/坐标/健康/状态机，避免改旧ExternalNav |
| `src/indoor_nav/scripts/relative_mission_manager.py` | 任务意图、只读FC核验、服务/事件/证据，不直发FC目标 |
| `src/indoor_nav/scripts/v0_nav_health.py` | 唯一受控FC出口、独立健康/租约与可选健康BRAKE退出 |
| `src/indoor_nav/config/mission_v0.yaml` | 用户可编辑任务及全部可配置健康限制；保留样例兼容默认值 |
| `src/indoor_nav/config/v0_sitl_gps.parm` | 独立默认GPS/EKF3、保守速度参数，仅SITL |
| `src/indoor_nav/launch/indoor_nav_v0.launch` | 默认只读、仅导航节点，不包含MAVROS/传感器 |
| `src/indoor_nav/CMakeLists.txt` | 仅增加两个Python节点安装 |
| `src/indoor_nav/package.xml` | 明确新增使用的geographic_msgs运行依赖 |
| `tests/test_v0.py`、`tests/test_v0_gate_ros.py` | 坐标/schema/状态机/租约/ROS gate检查与模式限制 |
| `tools/test_v0_relative_waypoints.py` | 离线/ROS只读smoke |
| `tools/v0_sitl_instance.sh` | 新runtime/独占端口的GPS或旧ExternalNav回归启动 |
| `tools/v0_sitl_test.py` | 多重身份门禁的仿真专用飞行、模式/链路/并行故障测试 |
| `tools/v0_sitl_stop.py` | 只停止已验证且DISARM的本轮模拟器及启动器 |
| `tools/v0_sitl_suite.py` | 每案例独立冷启动、机器可读退出/端口清理索引 |
| `tools/v0_dataflash_altitude.py` | 标准库离线读取高度/home证据，无飞控连接 |
| `tools/v0_archive_evidence.py` | 离线生成矩阵/索引/manifest，保留原始FAIL |
| `tools/v0_mock_acquisition.py` | 合成PCM大小+序号负载，不改真实采集驱动 |
| `tools/v0_analyze_bag.py` | 离线逐条读包、消息/序号/时间/误差统计，不回放控制 |
| `docs/v0/{README,00_REPO_AUDIT,SITL_REPORT,REAL_AIRCRAFT_SOP,SAFETY_AND_LIMITATIONS,FAKE_GPS_INTERFACE_GAP}.md` | 运行/审计/实测/真机门槛/限制/接口缺口 |
| `docs/v0/evidence/` | 全部PASS/FAIL、参数/YAML/源码hash、日志和manifest；大二进制按原gitignore留本地 |

未改旧mission/supervisor/estimator、旧launch/config、声学工作区、历史证据。完整已提交路径可用 `git diff --name-only 92f6e72 HEAD`；每个证据文件列于manifest。
