# ArduCopter V0 相对航点

**默认只读；不具备真机自动航点放行资格。F2 接口尚未确认。**

这是 `indoor_nav` 包的增量模式，不取代旧 ExternalNav。唯一业务控制出口为 manager 的 `/mavros/setpoint_position/local`。watchdog 仅观测、报告和撤销 manager 的输出许可，**不能在 manager 死亡时刹停飞机**。不请求模式、不 ARM/起降、不写 FC 参数、不管理 rosbag/音频进程。

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

启用控制需同时 `enable_control:=true dry_run:=false`；**两把开关**，不提供绕过健康检查。此命令不等于准许实飞；先完成 SOP 所有放行门槛。默认 Fake GPS 配置还要求真实上游健康接口。导航只读时不创建 MAVROS 控制 publisher；target_pose 仅为预览，不等于 FC 执行。

## F1 隔离仿真（自动 ARM/起飞只在专用测试程序中）

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
```

测试必须核验运行目录、SITL 可执行文件/PID、MAVProxy 唯一路由和 loopback socket、MAVROS arming 服务归属和 SIM 参数；不匹配直接拒绝。`multi` 起飞后先用仿真专用 yaw 命令改变初始航向，再冻结任务轴，含正 Z 点。F1 使用 **SITL 默认 GPS→EKF3**；不是 UWB/Fake GPS。F2 **没有验收命令**：接口未知，参见缺口文档，禁止用 F1 命令改名冒充。

## 配置契约

默认 `src/indoor_nav/config/mission_v0.yaml`，兼容用户样例。严格拒绝重复 YAML key、未知字段、缺字段、重复 ID、NaN/Inf、布尔代替数字、空航点、负时间、极端值与超界点。所有点为米、相对唯一启动 P0/yaw0；`(x,y,z)` 为前/右/上，不累计、不随当前机头旋转。`z=0` 保持启动 EKF 高度，不是 AGL。逐点 yaw 不支持，误填会拒绝；默认始终保持 yaw0。

四元数归一化、拒绝零范数；角度跨 ±π 正确处理。目标 ENU，右向先视为 FLU 负 Y，由 MAVROS 做 NED 转换，不在业务层重复换轴。

全部安全缺省值见 `indoor_nav/v0.py:SAFETY_DEFAULTS`，可在 `safety` 内显式覆盖：state/GPS/EKF新鲜度、速度/pose新鲜度、跳点、持续超速、悬停窗口/范围/角度、initial hold、ROS/墙钟偏差、周期/watchdog期限、冻结一致性。限制不是物理防护。软件 fence 检查所有目标和飞行当前位置。小于检测门槛的原点变化仍可能漏检。

`speed_xy_mps` 是**对人工预设 FCU 限速的上界要求**，不是插值速度控制：Copter4.5.7 必须 `20 <= WPNAV_SPEED <= 100*speed_xy_mps`。速度/垂直/加速度/Guided选项/源参数定期只读复核；缺失、过期、未知固件拒绝开始/撤销任务。UP/DN默认≤20cm/s，ACCEL默认≤100cm/s²，GUID_OPTIONS=0，EKF3/源GPS必须匹配。实际速度持续超限中止；短尖峰不立即 freeze。真机不自动改参数，未知新固件需新增审计和实测 profile。

## 服务与状态

```bash
rosservice call /v0_nav/start '{}'  # 飞手人工悬停、GUIDED、健康与限速检查后显式授权
rosservice call /v0_nav/abort '{}'  # 锁存、停止输出；不保证取消飞控最后目标！飞手接管
rosservice call /v0_nav/stop '{}'   # 仅 FINISH_HOLD→STOPPED，不切模式
rosservice call /v0_nav/reset '{}'  # 仅连接正常、DISARM、非活动状态；仍须再次显式 start
```

状态：`IDLE→WAIT_FOR_READY→CAPTURE_ORIGIN→HOLD_INITIAL→EXEC_WAYPOINT→DWELL→FINISH_HOLD`；完成持续发最后目标直至人工切走模式/stop。异常 `ABORT_LATCHED`，无自动续飞，无自动抢回 GUIDED。初始先发布冻结当前点并 hold，只有飞手预先进入 GUIDED且持续回读正确才推进；不实现程序切 GUIDED。双 start 拒绝。到达须 XY/Z 同时连续满足 stable_time_s，DWELL 离开容差则重新稳定，超时取消全部后续点。

定位坏时绝不把“当前位置保持”称为安全；本实现统一停发并明确 RC/FCU 责任。位置 setpoint 停发后 FCU 可能继续追踪旧目标。没有默认 RTL/LAND，真正备用模式必须现场验证。

## ROS 接口与证据

| Topic | 类型 | 内容 |
|---|---|---|
| `/v0_nav/status` | std_msgs/String | 10Hz小型JSON：state/mission_id/wp/reason/health/gate/control_enabled/pose_age/最大publish间隔/证据路径 |
| `/v0_nav/event` | std_msgs/String | 稀疏JSON：seq/mission_id/wall_s/ros_s/mono_delta_s/事件与状态；开始含固件与参数快照 |
| `/v0_nav/origin_pose` | geometry_msgs/PoseStamped | latched，冻结 P0/yaw0，map |
| `/v0_nav/target_pose` | geometry_msgs/PoseStamped | 预览目标，dry_run也可有，不代表飞控回执 |
| `/v0_nav/heartbeat` | std_msgs/Header | 10Hz活性，非控制 |
| `/v0_nav/watchdog` | std_msgs/String | healthy/reason，墙钟心跳和ROS图独占检查 |
| `/v0_nav/upstream_healthy` | std_msgs/Bool | 外部真实adapter提供，Fake GPS模式必需；不能用常量true替代证据 |

订阅 State、map PoseStamped、指定 `velocity_local` TwistStamped、raw/fix NavSatFix、EstimatorStatus。读取 ParamGet/VehicleInfoGet。GPS fix本身不能证明 UWB 链健康，仍要求上游。冻结检测仅在窗口位置近乎不变**且速度持续显著非零**时触发；不能识别所有同源共同冻结。

每次实例创建独立证据目录：原始YAML、绝对路径和SHA256、Git/dirty、源类型、固件版本/FC参数、wall/ROS/monotonic事件。默认 `~/.ros/v0_nav`，可 `evidence_dir:=...`。任务节点只持久化稀疏事件，遥测由已有采集系统录制；测试驱动另存实际位姿/真值/目标/速度/模式/echo/bag。ROS时间用于对齐，monotonic用于期限；主循环用墙钟等待，暂停 `/clock` 不会让 watchdog 静止。

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
# 仅在没有实机/导航订阅的隔离离线 Master 回放；不要向实机 Master rosbag play 控制话题
```

细分实测与未完成项见 `SITL_REPORT.md`、安全责任见 `SAFETY_AND_LIMITATIONS.md`。
