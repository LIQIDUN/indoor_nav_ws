# ExternalNav 协方差与融合核对（2026-10-09）

本轮仅完成源码审计、离线复现和默认关闭的 SITL 适配。没有修改飞控参数，没有新 DataFlash 飞行证据，不能声称在线 PErr 已下降或融合已改善。

## 真实数据路径

- world_estimator：位置在 nav/ENU，姿态为机体 FLU 到 nav；Odometry child 为 base_link。默认位置 XY 方差至少 0.01 m²，Z/姿态方差 1000；twist 保持原有零值和大方差。新增速度只有影子话题，未接入 ODOMETRY。
- 本机 MAVROS 1.20.1，安装目录没有插件 C++ 源码，因此核对官方同版本 [odom.cpp](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/odom.cpp#L180)：父坐标变换到 odom_ned，子坐标变换到 base_link_frd；位置在父系、速度在子系；两组 covariance 经 6×6 旋转，打包上三角；ODOMETRY frame 为 LOCAL_FRD/BODY_FRD，时间来自 ROS header。坐标变换见官方 [ftf_frame_conversions.cpp](https://github.com/mavlink/mavros/blob/1.20.1/mavros/src/lib/ftf_frame_conversions.cpp)。本地真实 ROS 回调测试验证 world→nav 位置路径、速度纯旋转及 FLU/FRD 符号。
- 本机 `/home/liqidun/github/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3659` handle_odometry：只接受 LOCAL_FRD/BODY_FRD；3682 行计算 `cbrt(cov[0]^2+cov[6]^2+cov[11]^2)`，这些是 XYZ 方差，不能误当标准差做常规 RSS。首元素 NaN 表示未知，posErr 保持 0。速度用姿态从 BODY_FRD 旋转到 NED；该路径不读取 velocity_covariance。
- `AP_VisualOdom/AP_VisualOdom_MAV.cpp:33` 位置误差下限 VISO_POS_M_NSE、上限100；70行速度误差来自 VISO_VEL_M_NSE，不来自 ROS twist covariance。
- `AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp:533,545,701` 分别选择外部位置/速度和把位置噪声钳位到 0.01..10 m 后平方。VISP.PErr=100 不代表 EKF 最终使用100 m标准差，当前链路最终最多10 m。
- 同文件1207、1261行，只有 activeHgtSource=EXTNAV 才使用外部 Z；POSZ=1 是 baro，另外需审查 range 高度切换配置，不能单凭 POSZ1 排除所有高度源切换。
- `AP_NavEKF/AP_NavEKF_Source.h` SourceXY::EXTNAV=6；`AP_NavEKF_Source.cpp:168,187` 可合并多个源集合的速度，EK3_SRC_OPTIONS 的 FUSE_ALL_VELOCITIES 默认开启。因此必须检查三套 VELXY/VELZ，而非只检查 SRC1。

## PErr 复现与修复边界

| 输入 | 离线结果 | 含义 |
|---|---|---|
| 方差 .01,.01,1000 | cube-root 后约100，VISP钳位100 | 不可信 Z 连带削弱 XY；与旧飞行日志 PErr100 相符 |
| pose covariance 首元素 NaN，FC噪声 .30 m | FC fallback .30 m | 协议未知哨兵，不是伪造 Z 精度；只在已核对位置模式下使用 |

新增 PositionOnlyAdapter 默认关闭，只有 SITL profile、MAV_TYPE_QUADROTOR、ArduPilot4.5.7、可读取 SIM_SPEEDUP，首次连接且 disarmed 才校验。只读取 vehicle_info/param/get，绝无 setter。校验 GPS_TYPE0、EKF3、VISO3、SRC1 POSXY6/POSZ1/YAW1、三套无 ExternalNav 速度/高度/yaw（也拒绝 yaw7）、VISO 安装偏移为零。每3秒更新只读缓存，5秒过期即停止该适配输出；VISO_POS_M_NSE 必须覆盖当前 XY 标准差和配置最低值，否则停止输出并诊断。

适配仅在发送副本的 pose covariance[0] 写入 NaN；内部 ROS 原始 Z 方差仍1000。MAVROS dense rotation 会传播 NaN，首上三角仍未知，已使用本机 Eigen 编译测试验证。FC 最终退回配置的单一噪声下限，不能保留各轴/逐样本协方差，这是明确的精度限制。不能把该方法泛化到其他固件或 ExternalNav 高度/速度模式。

替代路径 VISION_POSITION_ESTIMATE 在 GCS_Common.cpp:3709 起使用相似三轴合成，单纯换消息不能解决；把无测量根据的 Z 方差改小也不接受。

## SITL 建议与回滚

`config/sitl_external_nav_experiment.yaml` 和 `indoor_nav_sitl.launch` 的 `sitl_covariance_adapter` 默认 false。待操作员批准重启且确认已落地解除解锁后，才可加载 true 做无解锁对比；该参数不是 FC 参数写入。若只读校验失败，适配阻止发送，先查看 localization_diagnostics 的 adapter 状态，不自动修改参数凑条件。

当前建议保持 SRC1 VELXY=0、VELZ=0、POSZ=1、YAW=1。虽然本版本 ExternalNav VELXY 枚举确实为6，本轮不提供自动启用脚本：现有 twist 是零，派生速度有相关性和延迟，且没有可信 VZ/完整三维 body velocity，因此不具备安全接入前提。VISO_POS_M_NSE 候选 .20..5 m（必须覆盖实测误差及每包 XY 不确定度），默认最低 .20 只是保守试验门限，不是精度结论；VISO_VEL_M_NSE 在速度未启用前不改。

回滚只需在下一次经批准重启时保持 adapter=false；影子模块不会改变 EKF 源。禁止用清理工作树方式回滚用户修改。

## 对比与待验证

离线旧 PErr100→fallback .30 的算式与传输验证通过。在线 PErr下降、位置创新、观测接受、速度偏差、静止漂移、方向、源切换与融合改善全部未验证。新会话须同时记录 VISP/XKF1、位置创新日志、实际三套源参数、Gazebo、accepted UWB 与 shadow状态；仅看到话题发送不能认定融合成功。硬件与 Mock 噪声不得混用。
