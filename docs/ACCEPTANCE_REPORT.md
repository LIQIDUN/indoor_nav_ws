# 修复验收记录（2026-10-09）

基线提交 d1d99c8。实际任务书文件为根目录 CODEX_INDOOR.md，未发现用户所述长文件名。开始时已有 localization.yaml、ardupilot_expected.yaml、world_estimator.py 未提交修改，以及 CODEX_INDOOR.md、README_COLD_START.md、tests/、set_ekf_origin_once.py 未跟踪；全部保留，未 commit，未 ARM/TAKEOFF、未写飞控参数。

## 根因与改动

历史 XY_OUTSIDE_FENCE **尚未取得当次运行时参数证据，不能确认加载错配置**。代码可确定：world (0,0) 在 SITL XY 范围内，在实机范围外；当前 SITL include 原本正确选择 safety_sitl.yaml。普通 launch、残留参数/其他启动覆盖是待验证来源。新版本通过 profile/expected_profile、frame 一致性启动检查、config_source 和节点实际配置 JSON 提供可复现定位；不改变任何围栏边界。

确认的漏洞是旧逻辑对非任务飞行阶段放过 Z 越界，包括超过 z_max 和部分 armed 状态，以及 NaN Z。core.fence_reason 现在只允许 DISARMED 且 inactive 的地面低高度例外；safety_supervisor 拒绝非法 frame，并保留 ACK/reset/abort/LAND 语义，ACK 响应说明下一项阻塞。

SITL launch 原本显式关闭飞控参数检查，现移除覆盖，并独立使用 ardupilot_sitl_expected.yaml，避免仿真被实机 MTF 参数阻塞。普通 launch 保持实机期望配置。两份 safety YAML 添加 profile/frame，未变更实机数值。

Mock 原本独立订阅 /gazebo/model_states，model iris，从未读取 FC pose 生成 UWB；Range 仅高度。修复真值停止更新后无限重新发布旧坐标的漏洞，0.3 秒无新真值即停发。显式 freeze 注入仍用于故障测试，静止本身不应判定冻结。

保留用户已有的 estimator 独立 IMU/Range 校准和 ExternalNav 冷启动改动；补正 nav_from_world 的 Z translation，令其与配置 nav_z_offset 一致。Odometry XY 有方差下限，Z/姿态/twist 未观测分量使用 1e3；不能仅凭协方差认定 FC 忽略对应维度，必须核对源选择和日志。

mission_manager 保持服务接口，拒绝时带 safety reason；更正已过时的高度注释。新增三入口脚本和只读检查器，检查节点、服务、参数、origin、配置快照、任务目标、真值订阅、TF 可解析；不调用有副作用的服务。smoke 的动态部分为实际回调测试桩，在线部分为 DISARMED 快照，并非完整 Gazebo→FC 飞行验收。

## 验证与精确复现命令

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
python3 -m compileall -q src/indoor_nav/scripts tools/indoor_nav_check.py
bash -n tools/indoor_nav_preflight.sh tools/indoor_nav_status.sh tools/indoor_nav_sitl_smoke_test.sh
git diff --check
```

本次结果：catkin_make PASS；tests 10/10 PASS（未跳过）；core 9/9 PASS；Python 编译、shell 语法和 diff 检查 PASS。

实际 ROS 消息测试调用 MockUwb/WorldEstimator 的真实初始化、Gazebo 回调、generate/flush、校准及 publish；transport/timer/clock 使用桩，不连接真实 FC。连续参考标定超过 8 秒，**fc_pose 始终 None**；静止高度 0，缓慢位移 ΔX=0.10、ΔY=0.05 米在 world/odom 输出一致；真值丢失后 Mock 停止，定位失效、ExternalNav 停止。另有 π/2 旋转增量、围栏边界/NaN/地面高度、ACK/reset、任务未解锁/错误模式拒绝、跳点/超时、冻结检测及原数学测试。

在线检查尝试：`rosnode list` 返回 `ERROR: Unable to communicate with master!`，未发现 rosmaster/gzserver/arducopter/mavros 活跃进程。本机存在 ROS Noetic、ArduPilot SITL 二进制和 Gazebo world，但本次未建立全栈运行，未获得本次 GPS=0 的 FC 日志。运行 preflight 首次因沙箱标准 ROS 日志目录只读失败；获准在沙箱外重跑后，提示 Unable to register with master，10 秒限时退出 124，不能宣称在线 PASS。随后为预检加入初始化前 Master 探测，避免无 Master 时无限重试；修正后沙箱外重测立即返回 `FAIL runtime unavailable: [Errno 111] Connection refused`，退出码 1。

## 未完成验收与飞行结论

- GPS=0 全新飞控冷启动、origin 实际接受、ExternalNav 持续融合/创新量、FC 动态响应和约 0.2m 差值来源：**未验收**。
- 在线 TF 重复发布、历史围栏加载原因、实际参数持久化来源：**未验收**。启动脚本没有 --wipe，也不自动设置 GPS_TYPE，必须读回参数。
- 真值链动态与丢失测试为隔离回调桩，不是 Gazebo 物理仿真；未测闭环控制精度。
- mission stop/abort 自动 LAND 请求保留，但落地必须人工验证；服务返回成功不是落地证据。
- 现有任务 1.0m 航点距 SITL 围栏 0.1m，存在控制/机体裕量限制。禁止以扩大围栏消除风险。

目前可继续未解锁的仿真检查，**尚不能据本次结果认定可安全进行 SITL 航点飞行**。须完成操作说明中的 GPS=0 动态融合验收，再由操作员决定是否授权起飞。README_COLD_START.md 为用户已有历史说明，关于先用 GPS、FC pose 依赖等叙述以本次操作说明为准。

## 2026-10-09 后续冻结误报排查

已基于最新飞行 ROS 日志、DataFlash 00000343.BIN 及保留会话的只读在线数据完成专项排查与修复，详见 [FREEZE_INVESTIGATION.md](FREEZE_INVESTIGATION.md)。本次回放复现旧检测误报；新检测保留阈值与保护，用独立 Gazebo 运动证据区分静止与真冻结，并新增完整触发快照。当前 world_pose 停发直接原因已确认 FC_LOST；ArduPilot 处理停滞的具体内部原因、PErr=100m 导致的 EKF 质量问题尚未解决。此记录补充上次“无 Master”的验收结果；现在可访问用户保留的旧会话，但没有重启节点或进行飞行。后续测试：主测试 33/33、core 9/9。

## UWB 速度与协方差后续交付

本轮实现、离线结果及在线未验证项见 [UWB_EKF_ACCEPTANCE_SUMMARY.md](UWB_EKF_ACCEPTANCE_SUMMARY.md)。历史验收数字保留，不能用于证明新增适配已在线加载。

## MAVLink/ExternalNav 连续性验收

最新 [MAVLINK_EXTERNALNAV_FIX_REPORT.md](MAVLINK_EXTERNALNAV_FIX_REPORT.md) 记录了经操作员批准重启后的600秒真实无解锁验收：IMU/速度约25Hz、ExternalNav约30Hz、定位全程OK。此前报告中的“待批准重启/待在线连续性验证”是历史状态；协方差适配及EKF融合精度仍未验收。
