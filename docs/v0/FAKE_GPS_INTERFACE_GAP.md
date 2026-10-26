# F2 = BLOCKED：真实 Fake GPS 接口未取得

截至2026-10-26：在 indoor_nav_ws 和可读的开发机 ros_sensor 目录中未找到真实 UWB→Fake GPS 协议实现/设备映射。旧链路明确为 ExternalNav `/mavros/odometry/out`，不是 Fake GPS。F1 仿真内置 GPS 不构成等价注入链路。

## 需由机载人员提供（拆桨、只读调查）

1. 最新采集工作区路径、Git SHA/dirty、all_sensors.launch、auto_bag_recorder.py、IM72节点与话题/消息定义；只采集文本，不覆盖部署。
2. Fake GPS 程序绝对路径/版本/启动方式、输入 UWB 消息、输出协议/频率/单位/坐标/yaw/高度处理；GPS串口、GPS_INPUT、HIL_GPS 或其它必须有代码/抓取证据。
3. `ls -l /dev/serial/by-id`、`lsof /dev/ttyACM* /dev/ttyUSB*`、机载连线图、`rosnode list`、`rostopic list`、现有 MAVROS fcu_url。不可再开一个程序抢 USB/TELEM。
4. 只读 FC 固件版本/参数快照：GPS_TYPE*、SERIAL*、EK3_SRC*、EK3_ENABLE/AHRS_EKF_TYPE、高度/yaw源、Guided选项、速度/加速度、RC和失联failsafe。明确 cm/s 或 m/s。
5. 上游 sequence/source timestamp、valid、last genuine measurement、重启/坐标原点reset计数、质量与冻结判据；持续发送最后值不得被视为健康。

## 可替换 adapter 契约（尚未声称已实现真实适配）

- adapter 属于上游定位业务，不由导航 launch 启动。唯一职责为观测真实测量/注入状态；导航从 FC EKF local pose 取控制反馈，不再注入 GPS/ExternalNav。
- 输出 `/v0_nav/upstream_healthy`，`std_msgs/Bool`，建议≥5Hz；真实测量超时、故障、冻结、重启/reset必须 false；节点死亡触发导航1s新鲜度门槛。健康发布者不得从导航目标或 EKF pose 闭环生成自证。
- 布尔接口不能承载全部来源语义；真实 adapter 应另存原始质量/sequence/reset证据；未来若接 reset计数应直接锁存中止，不重新捕获 P0。
- 仅用于 SITL 的可替换输入为 `v0_mock_acquisition.py`（声学负载）与 F1 **内置 GPS**；它们不是 UWB→Fake GPS adapter。没有一个伪造 `healthy=true` 的“F2模拟器”。

## 解锁 F2 的步骤

确认协议后在全新 SITL runtime 走**同等协议**，单独配置 GPS/EKF、关闭默认 GPS/ExternalNav 冲突；用 FC 参数、EKF日志、真值误差证明采用该输入，而不是只看 local_pose。测试跳点、停流、序列停更但持续发送、时延、噪声、重启/原点变化、GPS与ROS/FCU断链，归档可机读结果。随后补接口等价脚本与至少一个故障注入通过记录。

没有 F2 PASS，不进入自动航点实飞；允许 R0 拆桨非飞行调查。不得写成“Fake GPS 已打通”或“可以真机放行”。
