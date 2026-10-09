# INVALID_VELOCITY_FRAME 修复报告

日期：2026-10-09。环境：ROS Noetic / MAVROS 1.20.1 / Copter SITL 4.5.7。

## 根因与配置证据

检查 [MAVROS 1.20.1 local_position.cpp](https://github.com/mavlink/mavros/blob/1.20.1/mavros/src/plugins/local_position.cpp)：

- `LOCAL_POSITION_NED` 的速度先 NED→ENU；`velocity_local.twist.linear` 使用该 ENU 向量。
- 该路径却将 `velocity_local.header.frame_id` 设为 `tf_child_frame_id`，默认 `base_link`（约第141–144行）。所以题述消息的数值是导航 ENU，不能根据错误 header 当作 FLU 再转一次。
- 同次回调构造的 `/mavros/local_position/odom` 使用导航父坐标系、机体子坐标系；其 twist 为 `R(q)^-1 v_ENU`，pose 中保留同一个 q（约第105–120行）。
- 实际 ROS 参数：`frame_id=map`、`tf/frame_id=map`、`tf/child_frame_id=base_link`，与已安装 apm_config.yaml 默认配置一致。配置本身没有把导航坐标系误设成机体系。`tf/frame_id` 是 TF 发布参数，不决定 velocity_local 的数值语义。

原监督器要求导航速度 frame 为 map/odom，拒绝 `base_link`，因此产生 INVALID_VELOCITY_FRAME。这一拒绝本身合理；问题是所订阅的 MAVROS 消息元数据不符合数值语义。

## 修复

新增 `src/indoor_nav/src/indoor_nav/navigation_velocity.py`；修改 `src/indoor_nav/scripts/safety_supervisor.py`。

监督器改为读取 `/mavros/local_position/odom`，以完整四元数执行 `v_ENU=R(q)v_FLU`，再按已有标定 yaw 的逆旋转转换到 world_lab，供冻结证据和预测围栏共同使用。这里是撤销 MAVROS 同次回调的编码，使用的姿态正是插件生成该 body twist 时使用的姿态，不是另外订阅的近邻 IMU，也不需要假定独立姿态消息与速度时间对齐。若未来更换发布者、插件改变这个配对关系，必须重新验证来源语义；不能将本转换套用到任意 Odometry。

严格校验父坐标系等于已配置导航系、子坐标系 base_link、四元数单位长度及全部数值有限。支持已验证的 map/odom 导航配置，拒绝其他配置。无效输入立即清除可用速度，诊断保留 `velocity_source_error`。不改 frame_id 字符串来解释原 velocity_local，不添加 base_link 白名单，也不依赖单独的 tf/frame_id 作为速度来源。

原始 measurement header 原样保留，receipt 单独记录，未来或超过原0.30秒窗口的速度继续被门禁拒绝。诊断快照新增 `velocity_source` 和 `velocity_source_error`；已有导航/世界速度、消息年龄、冻结窗口和触发条件保留。

冻结检测阈值、预测围栏范围/时域、FC_LOST 等门禁未放宽。MAVLink 25 Hz、ExternalNav 30 Hz、原IMU时间戳处理、UWB影子速度保留；没有启用UWB速度融合，校准前只读确认 `EK3_SRC1_VELXY=0`、协方差适配关闭。无飞控参数写入，也无实机配置修改。

## 自动化验证

新增 `tests/test_navigation_velocity.py`，扩展 `tests/test_freeze_ros.py`：

- yaw=0°/90°/180°/任意角；roll、pitch 对水平/垂直速度投影；组合姿态正反变换和幅值保持。
- 非法父/子坐标系、配置、NaN及非单位四元数拒绝。
- 实际监督器回调保留时间戳，未来/过期数据拒绝，无效新数据清除旧速度。
- 解码后的速度经标定旋转确实触发预测围栏；原冻结触发、LAND门禁等回归测试继续通过。

结果：67项主测试、9项核心测试全部PASS，无skip；catkin_make、compileall、git diff --check通过。验证命令和结果见 `evidence/velocity_frame_validation.txt`。

## 无解锁在线验证

操作员明确确认已落地、已解除解锁。检查时原ROS/MAVROS/ArduCopter/Gazebo进程已经退出；沿用本会话已有SITL重启及重新标定授权启动新会话。校准前再次检查连接、未解锁、Gazebo落地姿态及上述配置。校准成功，全程没有ARM、TAKEOFF、切GUIDED、启动任务或飞控参数写入。

`tools/indoor_nav_velocity_frame_audit.py` 为新增只读采集工具；30.186秒原始结果见 `evidence/velocity_frame_online.json.gz`，汇总见 `evidence/velocity_frame_online.summary.json`。

| 测量项 | 结果 |
|---|---|
| Odometry频率 | 24.994 Hz |
| 同header时间戳的odom / velocity_local配对 | 754对 |
| 解码ENU与velocity_local最大分量误差 | 1.39e-16 m/s |
| 速度header最大年龄 | 0.0451 s |
| 冻结分类 | WINDOW_INCOMPLETE 6次；STATIONARY 599次 |
| INVALID_VELOCITY_FRAME / POSSIBLE_UWB_FREEZE | 均0次 |
| MAVROS状态 | 所有采样connected=true、armed=false、STABILIZE |
| 安全门禁 | OPERATOR_NOT_ACKNOWLEDGED，605次；没有代替操作员确认 |

与Gazebo对照：新复位时Gazebo世界轴与导航ENU对齐，按接收时间取最近真值样本，Gazebo速度最大约1.32e-6 m/s，EKF导航速度最大0.1314 m/s，两者三维误差RMS为0.0940 m/s。静止试验只能验证零运动情况下的估计偏差，不能验证运动方向。配对消息几乎零误差证明转换准确撤销插件编码，**不代表EKF速度等于真实速度**。仍需操作员决定的后续动态实验检查方向和幅值，不建议据此直接启动航点任务。

补充60.169秒只读连续性回归（`evidence/velocity_frame_continuity.summary.json`及对应jsonl.gz）：IMU 24.991 Hz，最大间隔46.06 ms；ExternalNav 30.001 Hz，最大间隔36.07 ms；world_pose 30.000 Hz。此项为修复后的短时回归，不替代先前MAVLink报告的10分钟验收，也不能替代动态飞行验收。

## 尚未解决的问题及下一步

- 当前无飞行/平移真值激励，尚未完成动态Gazebo速度方向、幅值验证。不能宣称飞行验收完成。
- 静止时EKF速度存在约0.094 m/s RMS偏差，需结合EKF状态、定位噪声及原点/标定继续调查。保留SITL独立Gazebo运动证据，不能单凭EKF非零速度判UWB冻结。
- 重启后操作员确认门禁被重新置为未确认，这是预期安全行为；本次没有调用任务启动服务验证READY。
- 下阶段先检查+X/+Y标定和速度方向，再由操作员决定飞行；本次只证明坐标错误修复及无解锁回归结果。

## 回滚

不要 `git reset --hard` 或覆盖此前未提交修复。仅撤销本次 safety_supervisor.py 的 Odometry订阅、解码回调、严格导航frame及新增诊断字段，恢复原velocity_local订阅；删除新navigation_velocity.py、对应新增测试及只读采集工具（报告可留存）。这会恢复已知INVALID_VELOCITY_FRAME，不能作为飞行方案。操作员确认落地未解锁后再重载节点；重新进行标定和人工安全确认。无需回滚任何飞控参数，本次没有修改参数。
