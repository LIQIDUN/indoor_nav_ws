# 真机拆桨联调与首次小位移 SOP（本轮不执行）

## 放行门槛

**本轮授权仅为本地开发和隔离SITL，没有真实飞机操作授权。** 本清单是未来由人执行的操作文件，不是自动放行证明。F2专项模拟及协议调查已取消；最新普通GPS/GUI仿真结果及保留的失败见 `FAST250_DYNAMIC_SITL_REPORT_20261028.md`。R0实物签核、RC/飞控应急与真实采集检查未完成前，不进入自动航点飞行。SITL脚本绝不能在机载运行。

飞手独占ARM、起飞、模式开关、接管和降落职责；导航操作员负责只读核对/YAML/hash与显式start/abort；观察员独立监视净空、人员和定位质量。提前约定“停止”口令、备用模式及撤离路径。实体隔离区、护具和人员净空由现场负责人确认，软件围栏不包含旋翼/机身尺寸，不能代替物理防护。

## R0 拆桨、只读接口

1. 拆除全部桨叶，固定机体，确认电源/急停/RC，记录硬件和FC固件Git/参数快照。
2. 启动机载**最新版**采集入口；检查只有一个Master/MAVROS/FCU串口占用者，保持音频协议不动。确认实机 `/use_sim_time` 非true，不继承SITL环境。
3. 按用户确认的既有UWB→飞控GPS接口部署，检查定位质量、方向、静止稳定性、GPS/EKF健康及融合状态；记录现场可获得的质量/状态证据，不再调查底层协议或开发注入程序。禁止创建第二注入源。
4. V0默认只读启动；检查pose/速度frame、接收时间、原始stamp诊断、GPS/EKF有效性；手持前/右/上验证符号与yaw；观察重启/跳点/断流时健康拒绝或锁存，不为测试而ARM，不重新捕获原点偷偷续飞。pose/velocity/command样例超时0.5秒，state为3秒；按实际频率审核，不用改写stamp或放宽门槛掩盖停流。
5. 拆桨核对RC从GUIDED切至备用模式的回读及GCS/RC/EKF failsafe配置；不能以无动力回读代替实际高度/姿态保持验证，带动力行为留给后续经授权的安全阶段。室内不默认RTL，没有桨也不自动ARM。
6. 只读核验速度/加速度参数及单位，与实际目标固件对照。生产V0不会替人配置；任何参数修改应另行批准、备份、复核。

### R0 可复制的只读核对命令

以下只在现场另行授权后执行；不启动第二个MAVROS或Master。先由**机载已验证最新版采集入口**提供唯一MAVROS，两个业务shell使用同一Master及正确overlay：

```bash
printf '%s\n' "$ROS_MASTER_URI"
rosnode list
rosparam get /mavros/fcu_url
rosparam get /use_sim_time
rostopic type /mavros/local_position/pose
rostopic type /mavros/local_position/velocity_local
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/global_position/raw/fix
rostopic echo -n 1 /mavros/global_position/gp_origin
rostopic info /mavros/setpoint_position/local
rostopic info /mavros/setpoint_raw/local
ls -l /dev/serial/by-id
lsof /dev/ttyACM0
```

`/use_sim_time`缺省或false才可；不得继承11421仿真Master、SITL fcu_url或仿真配置。串口路径以现场已验证连接为准，`ttyACM0`只是示例，不换线、不启动第二串口程序。确认GPS fix、EKF健康及local pose持续更新，不能只凭fix有效判定真实UWB质量。

```bash
source /opt/ros/noetic/setup.bash
source ~/github/indoor_nav_ws/devel/setup.bash
roslaunch indoor_nav indoor_nav_v0.launch enable_control:=false dry_run:=true
# 另一同Master shell：
python3 tools/test_v0_relative_waypoints.py --ros-smoke
rostopic echo -n 1 /v0_nav/status
rostopic echo -n 1 /v0_nav/watchdog
```

R0全程拆桨、DISARM，不为了测试start而解锁。手持前/右/上移动，核对local位姿方向与yaw、零位移静止噪声、时间戳、姿态归一化、重定位/断流后的拒绝状态。任务P0/yaw0的实际冻结在R1/R2授权start时核对；DISARM下start本应被拒绝。静止不动不能当作冻结故障证据；下游位置/速度共同错误且自洽可能漏检，必须结合真实定位系统的可用状态和物理观察。任何姿态/轴向不明、跳点、EKF原点异常或健康判据不能解释，停止升级。

只读ParamGet记录4.5.7的WPNAV_SPEED、UP/DN、ACCEL/ACCEL_Z、GUID_OPTIONS、EKF/GPS源及RC/GCS/EKF failsafe；固件不同或参数单位不明即停止。0.20m/s为期望巡航，不是瞬时硬上限；监测阈值和实体净空不因观测到峰值而放宽。示例仿真FS_GCS_ENABLE=0不能复制为真机应急配置。

默认 `exit_action: manual_takeover` 不切模式。如拟启用 `brake_if_position_healthy`，须另行人工批准并核对BRAKE可用、独立健康条件和mode回读；定位失效时不依赖BRAKE，MAVROS或gate死亡时依赖飞控侧已验证failsafe和RC接管。拆桨只读检查不等于已验证带动力刹停性能。本轮Gate死亡及pose断流后实测仍有约0.22m/s真值速度，任务锁存/停发绝不等于刹停；若无法通过独立飞控/RC手段满足现场应急要求，禁止升级到R1/R2。

## R1 人工低空悬停 + dry_run

仅在对应仿真项目通过、R0检查及人工放行后。隔离场地、护具/围挡、合格飞手与观察员；先人工验证原位置/高度保持、Fake GPS质量、RC接管。V0 `enable_control:=false dry_run:=true`，核对冻结原点/yaw、目标在真实安全包络内，不发送运动目标。测pose age/循环抖动及CPU满载分布，人工确定门槛，不因误报随意扩大围栏。

## R2 单点

先结束R1并由飞手落地DISARM，再在地面启动控制版V0，让gate观测到IDLE/DISARM，然后由飞手人工起飞。不要在R1悬停中重启gate并期待接管；半空重启/失效后须落地重新授权。

飞手人工ARM/起飞/悬停并切GUIDED，确认备用模式可随时切。审阅仅一个0.2–0.3m点、z=0、速度已手工限至审计值；显式开启两把控制开关，明确调用start。观察真实运动/速度/模式；异常立即RC接管，abort撤销后续任务并由gate依配置处理，不当作已刹停。导航不自动LAND/DISARM。

首次只用一个独立YAML短前向点；不要直接加载默认五点方形。确认目标相对任务起点而非前一航点，`z=0`不是地面高度；手工悬停高度及机头方向由飞手按现场净空选择，冻结后不可重新捕获原点续飞。

```bash
# 仅在R0/R1签核、现场另行授权后；路径须为已审核的单点YAML
roslaunch indoor_nav indoor_nav_v0.launch mission_file:=/absolute/path/to/reviewed_single.yaml enable_control:=true dry_run:=false
rosservice call /v0_nav/start '{}'
rostopic echo -n 1 /v0_nav/origin_pose
rostopic echo -n 1 /v0_nav/target_pose
# 异常时先由飞手接管，操作员同时取消后续任务：
rosservice call /v0_nav/abort '{}'
```

飞手切出GUIDED后，确认任务锁存、无自行切回GUIDED或续飞；AltHold依靠高度保持而非水平定位，Stabilize需要飞手直接控姿态/油门，必须提前训练并明确各模式使用条件，不把任一模式当通用安全兜底。结束由飞手人工降落/DISARM，任务自身不执行。

## R3 小方形/往返

R2多次重复通过后，逐步增加前/右/回原点，保留同一P0/yaw0。逐点连续稳定与停留时间、实际路径/速度、软围栏与墙壁安全余量都由飞手/观察员核对。结束保持最后目标，飞手切出GUIDED并人工降落。重新任务先DISARM与显式reset/start，不自动续飞。

## R4 满负载联合

保持XVF16kHz与IM72 48kHz、位宽/增益/分块/话题原样；ESC/振动/音频与ARM/DISARM录包同时运行。确认新增V0话题只追加到现有列表。测试导航停止不杀录包、单音频故障不抢模式；资源劣化触发导航自身健康撤销。MAVROS断连依靠预验证FCU/RC应急，不依赖还能通信的watchdog。

R0/R1先验证单采集和只读双业务；R2/R3通过后才逐级做真实满负载，不在首次短点同时引入未验证设备/参数。kill进程等故障注入优先留在SITL或拆桨台架，不在首次带桨飞行中注入。检查ARM起包、DISARM结束后bag完整可读，XVF/IM72/ESC/振动原话题没有被新列表替换。

```bash
rosbag info --yaml /absolute/path/to/closed_flight.bag
rosbag check /absolute/path/to/closed_flight.bag
```

禁止向连接真机的Master回放位置目标/状态/控制话题。发现录包未落盘、pose陈旧、控制周期异常、定位跳变、速度持续超限、RC接管失败、净空不足、无关人员入区或传感器负载损害导航，立即停止升级并由飞手接管/结束试验。

## 每次人工签核与归档

记录FC firmware/EKF源/UWB定位质量与方向检查结果、RC开关/备用动作、限速单位和实际速度、碰撞边界与人员分工、YAML hash/Git/dirty、bag/FC日志、pose age和setpoint间隔、开始/到达/异常事件。DISARM自动停包为原采集逻辑，不代表导航杀录制。任何失败立即停止升级；不以SITL PASS替代现场风险评估。
