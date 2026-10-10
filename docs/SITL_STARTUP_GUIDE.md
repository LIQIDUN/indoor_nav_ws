# 日常 GUI SITL 启动指南

更新：2026-10-09。仅用于本机 ArduCopter 4.5.7 + Gazebo11 仿真，不用于实机。原入口已补齐 run_06 的关键定位及航点配置，不需要日常运行隔离自动飞行工具。

## 1. 启动：终端一

```bash
cd ~/github/indoor_nav_ws
./tools/start_indoor_sitl.sh
```

保留原有 Terminator、Gazebo GUI、QGroundControl、MAVROS 和室内定位节点。等待输出 **SITL PREPARED**；准备失败时查看组件终端及`.sitl_runtime/gui_instance/prepare.json`，不要飞行。

启动器现在会：

- 在工作区生成1 ms步长的world副本，不修改外部原world。
- 将原EEPROM复制到工作区GUI运行目录；不修改原EEPROM。
- 固定home为`-35.363261,149.165230,584,0`。
- 使用MAVProxy 25 Hz，并在身份检查后请求HEARTBEAT 5 Hz。
- 只读核实EKF3/ExternalNav位置配置；不符即报错，不静默改EKF源。
- 在本机SITL副本中开启完整ARMING_CHECK=1、LOG_DISARMED=1，记录前后值。
- 设置并回读EKF origin，核实经纬度及高度；不把“发送成功”当作接受成功。
- 显式开启SITL位置协方差适配，选择run_06的10航点。

它**不会校准、ACK、ARM、起飞或启动航点**。启动前必须确认旧实例已落地未解锁；不要同时运行隔离测试环境。

## 2. 校准与检查：终端二

```bash
cd ~/github/indoor_nav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_IP ROS_HOSTNAME

# 飞机在初始地面位置，机头朝world_lab +X，地面标签稳定约8秒后：
rosservice call /indoor_nav/calibrate
rostopic echo -n1 /indoor_nav/localization_status
rostopic echo -n1 /indoor_nav/world_pose
rostopic echo -n1 /mavros/global_position/gp_origin
```

应看到：

- 校准返回success=true；calibrated、transform_locked、healthy均为true，reason=OK。
- world_pose坐标系为world_lab，地面位置接近Gazebo初始原点。
- origin纬度-35.363261、经度149.165230、海拔约584 m（回读583.999 m正常）。

校准仍使用原服务：要求未解锁、FC连接新鲜、有效飞机UWB（0.20秒）、IMU接收/header（0.40秒）、Range接收/header（0.20秒）、地面标签约8秒稳定窗口及至少25个样本。地面标签偏移上限0.30 m、标准差范数上限0.10 m保持不变。无需先有EKF local_position，也无需先ACK。

重复校准返回Already calibrated代表已锁定；需要重标定时须落地未解锁后重启定位节点并重新检查。

可选短检查（每条Ctrl+C结束后执行下一条）：

```bash
rostopic hz /mavros/imu/data
rostopic hz /mavros/local_position/odom
rostopic hz /mavros/odometry/out
rostopic echo -n1 /indoor_nav/localization_diagnostics
```

IMU/FC odom约25 Hz、ExternalNav约30 Hz；诊断transport_adapter应为enabled=true、reason=CHECKED_POSITION_ONLY_UNKNOWN_COVARIANCE。适配重查可能造成单帧停发，不能仅凭频率证明EKF融合或飞行通过。

## 3. 正式确认、手动起飞与航点

人工核实上面的origin、仿真身份、初始姿态和定位健康，再执行：

```bash
rosservice call /indoor_nav/safety/acknowledge
rostopic echo -n1 /indoor_nav/safety_status
```

必须实际ready=true、无ABORT；ACK成功本身不保证READY。然后逐条执行并检查返回成功：

```bash
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'GUIDED'}"
rosservice call /mavros/cmd/arming "{value: true}"
rosservice call /mavros/cmd/takeoff \
  "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.8}"
```

解锁后仍在地面时可能出现Z_BELOW_FENCE；保留原门禁，不改阈值。到约0.8 m稳定悬停、实际GUIDED/armed=true且安全READY后：

```bash
rosservice call /indoor_nav/mission/start
rostopic echo /indoor_nav/mission_state
```

看到COMPLETE后Ctrl+C停止echo，再降落：

```bash
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'LAND'}"
rostopic echo /mavros/state
```

**mission/start不自动降落**。等待Gazebo实际落地且armed=false；不要在空中解除解锁。QGC用于观察时不要同时发送冲突的模式/任务指令。启动后QGC通常通过本机UDP14550自动连接，MAVROS使用14552，不能互相抢占端口。

## 4. 正常关闭

落地且armed=false后：

```bash
./tools/stop_indoor_sitl.sh
```

仍使用原关闭方式，不需要--force。仅关闭窗口或启动器不等于仿真已退出。下一次冷启动重新复制EEPROM；当前GUI副本内的参数调整不会写回种子文件。重新启动后须重新标定和人工ACK。

## 可选配置与隔离环境区别

tools/sitl.env提供显式选项，默认使用run_06配置：

```bash
# 默认true；仅诊断回滚时关闭，不能将关闭后的状态当作run_06配置
SITL_COVARIANCE_ADAPTER=false ./tools/start_indoor_sitl.sh
# 可选择其他任务文件
SITL_MISSION_FILE=/绝对路径/mission.yaml ./tools/start_indoor_sitl.sh
```

全局launch适配默认仍为false，仅GUI入口显式选择true；UWB派生速度仍为影子输出，不融合。实际EKF检查要求GPS_TYPE=0、AHRS_EKF_TYPE=3、EK3_ENABLE=1、VISO_TYPE=3、EK3_SRC1_POSXY=6、VELXY=0、VELZ=0、POSZ=1、YAW=1、零VISO位置偏移、VISO_POS_M_NSE≥0.20 m，并拒绝三套源中的ExternalNav速度/高度/航向融合。EEPROM不符时准备失败，需要先调查，不能关闭检查。

| 项目 | 日常GUI入口 | run_06隔离自动飞行环境 |
|---|---|---|
| ROS / Gazebo Master | 11311 / 11345 | 11321 / 11355 |
| MAVROS UDP | loopback 14552 | loopback 14572 |
| GUI / QGC | 启动；MAVProxy另输出14550 | 无GUI、无QGC，唯一输出14572 |
| world / EEPROM副本 | .sitl_runtime/gui_instance | .sitl_runtime/flight_instance |
| 定位 / 任务 | 相同适配、1 ms、25 Hz、5 Hz、固定origin、10航点 | run_06已飞行验证的配置 |
| 标定 / ACK | 手工服务调用 | prepare辅助标定，ACK仍由人工完成 |
| 飞行 / 日志 | 手动；FC DataFlash自动记录 | 工具检查身份并控制全程，另保存bag/轨迹/摘要 |

两者仍共用TCP5760、Gazebo9002/9003，不能并行启动。旧stop脚本进程匹配范围较宽，也不要用它关闭一个同时存在的隔离实例。

本次新增sitl_gui_prepare.py只是原启动器内部准备步骤，不是第三套启动器；现有自动飞行工具未删除。GUI环境没有自动飞行工具的每步身份复核、自动恢复或自动bag记录。关键配置一致不代表GUI全航线已再次通过，在线验证结果见下文。

## 本次验证

使用修改后的原GUI入口完成真实无解锁验证（QGC和Gazebo GUI进程均在运行），采集45.15秒：

| 指标 | 实测频率 | 最大消息间隔 |
|---|---:|---:|
| FC IMU | 24.986 Hz | 48.1 ms |
| FC local velocity | 24.987 Hz | 48.9 ms |
| 原始MAVLink HEARTBEAT | 4.998 Hz | 207.6 ms |
| world_pose | 29.999 Hz | 36.8 ms |
| ExternalNav odometry/out | 29.667 Hz | 67.1 ms |

校准成功且锁定，定位全窗口healthy=true/OK；安全状态OPERATOR_NOT_ACKNOWLEDGED、无ABORT，FC为STABILIZE且armed=false。DataFlash主循环399～401 Hz，VISP.PErr共5616条均约0.20 m；原world仍为2.5 ms，副本为1 ms。原stop脚本正常关闭，未残留仿真组件。

78项主测试、9项核心测试通过，Shell语法/Python编译/差异检查通过。新增测试覆盖错误Master拒绝、危险融合源在写入前拒绝、origin经纬度和高度匹配、非法home及参数日志字段。启动准备增加了MAVROS初次参数缓存的有界等待，未放宽数据新鲜度门禁。

**结论：关键定位及航点配置已与run_06一致；本轮通过启动/校准/频率/关闭验证，未执行GUI环境飞行，也未执行人工ACK后的READY验收。** 保留run_06已有飞行证据，不能把本轮静态验证写成新航线通过。已知ExternalNav单帧停发仍存在，没有重复发布旧数据掩盖。

证据：[频率汇总](evidence/sitl_gui/audit.summary.json)、[准备与origin回读](evidence/sitl_gui/prepare.json)、[状态快照](evidence/sitl_gui/snapshot.json)、[DataFlash摘要](evidence/sitl_gui/dataflash_summary.json)、[测试](evidence/sitl_gui/tests.txt)、[关闭记录](evidence/sitl_gui/shutdown.txt)。原始采集保存为audit.jsonl.gz。
