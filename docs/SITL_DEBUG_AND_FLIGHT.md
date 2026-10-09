# SITL 无 GPS 检查与人工飞行流程

仅适用于本项目 Gazebo Classic / Copter 4.5.7 SITL。所有航点是 `world_lab` 绝对坐标。脚本不会 ARM、TAKEOFF、调用 mission/start 或发送位置目标。首次飞行仍须人工批准，并完成下列动态验收。

```bash
cd ~/github/indoor_nav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
./tools/start_indoor_sitl.sh
./tools/indoor_nav_status.sh
```

启动器读取 `tools/sitl.env`，默认 ExternalNav 开启；无 `--wipe`、无自动飞控参数写入，沿用 ArduPilot 工作目录中的持久化参数。因此“重启了进程”不等于“已正确配置无 GPS 冷启动”。不要同时运行普通 indoor_nav.launch，后者默认加载实机围栏。

在 **SITL、DISARMED** 下由操作员在 QGC/MAVProxy 核对 `config/ardupilot_sitl_expected.yaml` 中全部参数（GPS_TYPE=0、EK3_ENABLE=1、VISO_TYPE=3、POSXY=6、VELXY=0、POSZ=1、VELZ=0、YAW=1、VISO_POS=0），如需修改，仅修改仿真，然后重启全新飞控进程并保存本次日志。不要复制到真实硬件。LOG_DISARMED/LOG_BACKEND_TYPE 由操作员核查开启，记录具体固件日志类型。

若无 EKF origin，在静止未解锁状态显式指定已知 SITL home 地理参考（不是 GPS 测量），然后确认收到 origin：

```bash
python3 tools/set_ekf_origin_once.py --lat <已知纬度> --lon <已知经度> --alt <已知AMSL米>
rostopic echo -n1 /mavros/global_position/gp_origin
# 机头沿 world_lab +X，等待地面参考 Tag 连续稳定至少 8 秒
rosservice call /indoor_nav/calibrate
./tools/indoor_nav_preflight.sh
./tools/indoor_nav_sitl_smoke_test.sh
```

preflight 是只读快照，PASS 不代表飞控已融合或获准飞行。人工核实 origin 接受情况后才 acknowledge。现有 SITL launch 的 `operator_verified_fixed_ekf_origin=true` 是仿真声明，不是 origin 存在的证据。该声明不得用于真实硬件。

```bash
rosservice call /indoor_nav/safety/acknowledge
rostopic echo -n1 /indoor_nav/safety_diagnostics
```

诊断 JSON 显示节点实际缓存的配置来源、profile、围栏、XYZ、frame、armed/mode、定位数据年龄。参数与节点快照不同必须未解锁重启节点，不能只改 rosparam 期待缓存自动更新。ACK 成功仅确认操作员，不保证 READY。

SITL XY 范围 [-1.10,1.10]，实机 [0.40,2.60]，两者 Z 范围 [0.30,1.30] 米，均未扩大。SITL (0,0,0) 仅在未解锁且任务未运行时允许低于 z_min；XY 永远检查，已解锁严格检查 Z。起飞阶段安全状态可能 WAIT，但没有启动任务，不会发航点。首航点 (0.5,0.5,0.8)，最大 XY 航点 1.0 距 SITL 上界仅 0.10 米，小于 0.15 米到达容差，必须评估机体尺寸和控制误差，不以此认定可飞。

动态验收使用隔离仿真或测试桩，不在真实机上发布真值输入：记录同步序列，缓慢沿 world +X/+Y 平移，对比 Gazebo、LinkTrack、world_pose、odometry/out 和 FC local_position 的增量、延迟与方向。ROS 测试桩已覆盖前四段，但不能证明 FC 融合。必须取得本次 GPS_TYPE=0 冷启动的 ExternalNav 使用日志和创新量/动态 FC 响应，不能使用旧日志或仅凭 30Hz 判定。可录制：

```bash
rosbag record -O /tmp/indoor_nav_disarmed.bag /gazebo/model_states /nlink_linktrack_nodeframe2 /indoor_nav/world_pose /mavros/odometry/out /mavros/local_position/pose /mavros/statustext/recv /indoor_nav/localization_status /indoor_nav/safety_diagnostics
```

坐标链（箭头是坐标映射，不表示这些都由本包广播为 TF）：

```text
world_lab -- Rz(calibration yaw), fixed XY/Z offsets --> odom (ENU)
odom -- swap X/Y, negate Z --> odom_ned (NED)
base_link (FLU) -- keep X, negate Y/Z --> base_link_frd (FRD)
```

nav_from_world 消息 parent=odom、child=world_lab，p_nav=R*p_world+t；任务使用同一变换，安全检查使用逆变换。静态 TF parent=odom_ned child=odom、parent=base_link_frd child=base_link；节点名称不表示 TF 方向。map 由 MAVROS/其他组件管理，本包不创建 map。不要重复发布这些静态边；预检仅验证可解析，重复发布者还须查看 /tf_static 发布节点。Z offset 是固定配置，不再来自 FC pose。约 0.2 米差值不得用 VISO_POS 或篡改世界坐标抵消。

完成动态验收、人工批准后，由操作员在 QGC/RC **手工 ARM、起飞至围栏内安全高度、进入 GUIDED**，再次检查 READY，才执行：

```bash
rosservice call /indoor_nav/mission/start
rosservice call /indoor_nav/mission/stop
```

start 不解锁、不起飞、不切模式。stop 停止任务目标，不代表已降落；已执行任务且仍 GUIDED 时 supervisor 会锁存 abort 并请求 LAND，必须核实实际模式和落地，失败时人工接管。RC 离开 GUIDED 优先，代码不会强制拉回。落地并 DISARM 后才可 reset，reset 清除 ACK，必须重新检查并确认。

```bash
rosservice call /indoor_nav/safety/reset
./tools/stop_indoor_sitl.sh
```

冻结报警的窗口、时间戳、只读捕获和本次飞行证据见 [冻结专项排查](FREEZE_INVESTIGATION.md)。升级时需同步重启 estimator/mock/supervisor；保持 `/use_sim_time=false`，安全节点单独监视 `/clock`，禁止用暂停仿真同时暂停安全计时。
