# UWB / EKF 修复验收（2026-10-09）

基线 master/c5baa43，开始工作树干净；保留前轮冻结保护、诊断快照和回放，不提交、不推送。本轮所有修改留在工作树。任务书作为需求依据；用户授权继续修改，不等于授权解锁飞行或重启遗留会话。

| 项目 | 状态 | 交付/边界 |
|---|---|---|
| A 驱动速度字段审计 | 已完成，实物证据不足 | [UWB_VELOCITY_CAPABILITY.md](UWB_VELOCITY_CAPABILITY.md)；原生默认禁选，Mock速度不冒充设备速度 |
| B accepted UWB 派生速度 | 已完成，仅离线验证 | velocity.py/velocity_ros.py；稳健窗拟合、去重、时间质量、杆臂、旋转、协方差、影子话题 |
| B 误差/延迟评估 | 已完成合成评估；待在线验证 | [UWB_VELOCITY_EVALUATION.md](UWB_VELOCITY_EVALUATION.md)、evaluate_uwb_velocity.py、evidence/uwb_velocity_offline.json |
| C PErr100来源/速度源审计 | 已完成源码与算式验证 | [EXTERNALNAV_EKF_FUSION_REPORT.md](EXTERNALNAV_EKF_FUSION_REPORT.md) |
| C 协方差修复 | 默认关闭，只有离线验证 | SITL-only未知哨兵适配，运行时版本/三套源/噪声只读检查；禁止假造Z精度 |
| C PErr下降/融合改善 | 待在线验证 | 不宣称已有实际改善，无新飞行日志 |
| D world_pose停止 | 已确认门禁原因 | FC_LOST/IMU过期停止输出，ABORT不直接停发布 |
| D SITL内部停滞根因 | 尚未定位；后续复现待批准 | [SITL_STALL_INVESTIGATION.md](SITL_STALL_INVESTIGATION.md)，长时只读采集工具已增强 |
| E1 构建/测试 | PASS | 50主测试+9核心测试，无跳过；详见下列命令 |
| E2 新节点无解锁联调 | 待操作员批准重启 | 遗留会话不加载本轮代码，断连 armed:false 不证明安全解除解锁 |
| E3 飞行验收 | 未执行，需单独批准 | 不因离线通过自动启动任务 |

## 修改范围

world_estimator 增加设备计数排序、accepted元数据、速度影子、适配接口；Mock增加模拟设备计数，vel_3d仍零。新增 uwb_velocity.yaml 只配置影子，sitl_external_nav_experiment.yaml 默认disabled。launch 加载这些配置，但 ExternalNav 转发原有默认false、协方差适配默认false。硬件 localization/safety/ardupilot YAML 未改；safety_supervisor和冻结阈值未改。

适配不写FC参数，所有新增FC服务只有 vehicle_info_get 和 param/get。没有ARM/TAKEOFF/GUIDED/mission-start，没有实机参数修改，没有新会话启动、重启、端口修改或debugger attach。只做了离线launch配置解析，true/false两种配置均通过；这不代表启动过节点。

## 验证命令与结果

在仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
python3 -m compileall -q src/indoor_nav/scripts src/indoor_nav/src tools tests
python3 tools/evaluate_uwb_velocity.py > /tmp/uwb_velocity.json
git diff --check
```

全部PASS；主测试50项，核心9项，无skip。覆盖旧343误报回放、真冻结与静止/陈旧真值区分、触发快照字段及LAND门禁；新增计数重复/乱序/回绕、接收倒退、gap/初始化、静止/冻结歧义、非有限/异常速度、Huber异常点、原生选择/证据/新鲜度、杆臂和ENU/NED/FLU/FRD方向、位置平移不进入速度、FC twist仍零、重复包不刷新定位年龄。Eigen C++测试编译后运行，验证MAVROS形式的dense covariance变换；只读适配校验测试覆盖版本、机型、初始disarmed、缓存过期、三套源和低噪声拒绝。

构建和测试运行记录见 `evidence/uwb_ekf_validation.txt`。离线launch解析首次因沙箱禁止网卡枚举失败，经工具审批在沙箱外重跑通过，没有启动ROS节点或访问FC。未运行在线预检/长监测，避免把遗留旧会话当新代码验收。

## 失败案例、限制与下一步

原生设备语义/实际滤波未知；接收时间拟合未解决设备同步及传输抖动。派生速度同位置相关，窗口延迟约.24秒，合成异常点最大误差可达.428m/s，不能用影子有效标志替代安全保护。完全静止与定位冻结只能结合独立运动证据区分。unknown covariance适配丢失各轴方差，依赖FC标量噪声覆盖真实XY误差，版本和源配置不符合时失败关闭。

在线验收应先由操作员确认落地解除解锁并批准重启，全套加载，保持速度融合关闭；先baseline再适配对照。记录实际参数、Gazebo/accepted/shadow/EKF、VISP/XKF1/创新与长时心跳队列，依据真实噪声制定精度指标。现有范围不能确认坐标方向闭环、PErr实际降低、融合接受率或停滞不再发生。

回滚：下一次获准启动时保持 sitl_covariance_adapter=false；若需撤销本轮实现，按上述文件范围逐项审阅diff，保留用户既有修改，不执行reset/clean。没有FC参数变更需要回写。在线和飞行步骤保留为待批准项，不越权补做。
