# V0-Research 本地验证记录（2026-11-03）

本记录区分离线软件回归、隔离SITL、测试探针失败及真实飞机未测项，不是部署/实飞放行单。

## 1. 基线与保护

- 分支：`feat/arducopter-v0-relative-waypoints`，没有切换或新建分支。
- 起始/结束HEAD：`dcf3919d6b14d5531e4044c892bc4572e8875ffd`。
- 工作区：`/home/liqidun/github/indoor_nav_ws`；`.git`只读，不提交、不push、不pull/reset/clean。
- 起始没有已跟踪文件修改；有59个未跟踪普通文件（Git短状态按文档/目录合并显示）。
  原有中文设计文档、配置指南、实机部署/人工审核文档及20261028证据全部保留。
- 原始严格YAML：SHA256 `f33c125f3c3e9b0e89050b4a6ee5d4ec6050bce4a8ced49181aa989b16efa21b`，字节未改。
- 初始清单、严格YAML哈希、59项原有文件校验和最终源码哈希放在 `evidence/v0-research-20261103/`。
- 历史报告/FAIL不改写；本次所有成功、失败和重试都保留独立目录。

## 2. 离线测试与构建

环境：Ubuntu/ROS Noetic，`/usr/bin/python3`，MAVROS 1.20.1。

| 项目 | 结果 | 说明 |
|---|---|---|
| tests全套 | PASS，179项，无跳过 | 包括真实ROS消息回调、Launch解析、既有功能及新增27项回归 |
| src/indoor_nav/test | PASS，9项 | 原核心回归 |
| V0专项子集 | PASS，101项，无跳过 | 是179项的一部分，不重复计入总测试数；可PYTHONNOUSERSITE=1运行 |
| 严格YAML离线smoke | PASS | 原文件/右向方形，SHA不变 |
| level YAML离线smoke | PASS | 左向定高方形，5点，独立哈希 |
| research YAML离线smoke | PASS | 4项科研策略、左向定高5点 |
| catkin_make -j2 | PASS | 新stream节点安装到devel Python relay；源脚本可执行 |
| 新部署文档Bash语法 | PASS，9块 | 仅bash -n，不执行机载命令；原README中的历史占位示例不当作可执行脚本 |
| git diff --check / 原有文件校验 | PASS | 保护原严格文件及59项起始未跟踪文件 |

实际执行的主命令：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
PYTHONNOUSERSITE=1 python3 -m unittest discover -s tests -p 'test_v0*.py' -v
PYTHONNOUSERSITE=1 python3 tools/test_v0_relative_waypoints.py --config src/indoor_nav/config/mission_v0.yaml
PYTHONNOUSERSITE=1 python3 tools/test_v0_relative_waypoints.py --config src/indoor_nav/config/mission_v0_level.yaml
PYTHONNOUSERSITE=1 python3 tools/test_v0_relative_waypoints.py --config src/indoor_nav/config/mission_v0_research.yaml
catkin_make -j2
```

**环境失败与保留记录：**首次把全套tests也设置PYTHONNOUSERSITE=1，旧flight runner测试导入mavros.mavlink时找不到pymavlink，失败1项导入。
本机已有用户级pymavlink，恢复用户要求的普通python3测试命令后全部通过；没有安装/替换依赖。
新stream生产节点不依赖pymavlink；它的专项测试及只读启动仍使用PYTHONNOUSERSITE=1。

新Launch测试最初把ROS remap_args的list与tuple比较，出现1项断言失败；修正测试期望类型，保留原日志。
没有为测试通过修改生产Launch语义。

倾角测试额外发现旧判据在恰好0.35rad时因归一化舍入误报。加入平方量绝对1e-12数值等价容差，
精确边界/缩放回归通过，0.350001rad仍拒绝；不是提高飞行倾角限值。

## 3. 覆盖内容

- strict不稳定悬停拒绝；research无历史也可在ARM/GUIDED/位置健康时start；DISARMED、非GUIDED、缺少位置或外部故障拒绝。
- P0/yaw0冻结、重复start不重置、四个基准yaw的FRU→ENU、z=0保持start高度、当前yaw变化不旋转任务轴。
- strict持续超速仍中止，research超速只记录一次且保留速度/峰值；strict超时中止，research等待且最终可正常完成/中止。
- 到达稳定、停留、中断停留、围栏、姿态、非法四元数、定位断流、目标超时、原点变化、GPS/EKF、时钟/控制租约与人工接管。
- 双节点同策略；持续冻结不一致仍硬阻断；性能参数只记录但接口/来源/未知固件仍阻断，参数快照去重。
- 五组stream的真实Noetic常量与频率、EXTENDED_STATUS=2、0Hz跳过、服务异常/部分成功重试、连接代次/重连、空响应无success字段。
- 默认只读、respawn=false、单MAVROS include、start_mavros=false无MAVROS、mission_file/evidence_dir/pose_topic透传。

## 4. 隔离动态SITL：11次飞行尝试，10 PASS / 1 FAIL

仅运行本机GPS→EKF3的Gazebo/ArduCopter仿真，ROS11421、Gazebo11455、MAVROS回环UDP14672，
每案例冷启动、独立EEPROM/runtime，验证可执行文件/PID/路由/SIM参数/服务归属后才由测试驱动仿真ARM/起降。
生产节点不ARM/起降。所有案例完成清理时均DISARM，端口释放。

| 独立证据目录（evidence/下） | 模板 / case | 结果 | 说明 |
|---|---|---|---|
| 20261103T132555-single | research / single | PASS | 0.25m单点、接管/无自动恢复 |
| 20261103T132812-multi | research / multi | PASS | 多点、非零初始yaw及正Z |
| 20261103T133046-abort | research / abort | PASS | 显式中止及条件化BRAKE反馈 |
| 20261103T133301-mode_exit | research / mode_exit | PASS | 飞手切出GUIDED优先、不得自动续飞 |
| 20261103T133514-manager_exit | research / manager_exit | FAIL | 注入Manager退出前真实pose接收间隔超时，未完成该次预定注入 |
| 20261103T133725-pose_loss | research / pose_loss | PASS | 定位断流撤权，无盲目BRAKE |
| 20261103T133940-gate_kill | research / gate_kill | PASS | 唯一出口死亡、Manager锁存；不证明物理刹停 |
| 20261103T134159-single | 最终research / single | PASS | 倾角数值修正后单点复验 |
| 20261103T134415-multi | 最终research / multi | PASS | 倾角数值修正后多点复验 |
| 20261103T134649-manager_exit | 最终research / manager_exit | PASS | 同一最终源码重试，正常退出/撤权/健康条件下BRAKE |
| 20261103T134906-single | 最终strict / single | PASS | 原严格模板兼容回归 |

首次manager_exit失败时，Manager观测pose age约0.508s已超过0.5s保护；测试驱动同话题实际最大接收间隔0.973590s，
velocity同步约0.973584s。不是仅header时间偏差；未定位该次仿真遥测停顿的根因。
不为通过而放宽断流阈值；失败后按测试程序降落/DISARM并清理。重试PASS不覆盖这次FAIL。

源码证据边界：前三次使用core SHA256 `4c932c3122233b658400e91f807ab9054bc309e0af39b660b0cc5e267e77df04`；
从mode_exit起使用最终 `d399ea87f1f9bec3e68fb93f73dbc352b4dd14ff84d54f13e91e63e1996cd5b2`。
两者差异仅倾角精确边界的数值等价处理，最终单点/多点均重测。每案例inputs.json保存当时实际源码/任务模板哈希。
测试模板会改为对应case航点、fc_ekf_sitl_gps与条件化BRAKE；不是生产5点YAML逐字原样的实飞或完整动态验收。

运行命令（分三批，不并行飞行）：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11421 ROS_IP=127.0.0.1
python3 tools/v0_sitl_suite.py --sitl --execute --mission-file src/indoor_nav/config/mission_v0_research.yaml \
  --cases single multi abort mode_exit manager_exit pose_loss gate_kill
python3 tools/v0_sitl_suite.py --sitl --execute --mission-file src/indoor_nav/config/mission_v0_research.yaml \
  --cases single multi manager_exit
python3 tools/v0_sitl_suite.py --sitl --execute --cases single
```

首批整体退出1（6 PASS/1 FAIL），后两批退出0。套件目录分别为
`resume-suite-20261103T132546-237930`、`resume-suite-20261103T134150-245361`及`resume-suite-20261103T134857-248492`。
机器可读总索引见本次证据目录的sitl-index.json。

## 5. 生产外层Launch的DISARM只读集成

最终探针 `evidence/20261103T141057-readonly-streams/summary.json`：**PASS，两配置均通过**。
level/research均加载正确任务路径、control_enabled=false、Watchdog健康、EKF健康、唯一MAVROS；
调用start均返回DISARMED拒绝，MAVROS位置目标发布者列表为空，未调用ARM/起降/切模式。
EXTENDED_STATUS/POSITION/EXTRA1/EXTRA3/RAW_SENSORS五组请求均由ROS日志证实发出。

每配置约20秒真实话题接收观察：

| 配置 | 话题 | 平均Hz | 最大接收间隔s | 消息数 |
|---|---|---:|---:|---:|
| level | pose | 24.981 | 0.0457 | 500 |
| level | velocity | 24.981 | 0.0457 | 500 |
| level | GPS raw/fix | 24.982 | 0.0456 | 500 |
| level | IMU | 24.982 | 0.0454 | 500 |
| research | pose | 18.431 | 0.1136 | 369 |
| research | velocity | 18.431 | 0.1136 | 369 |
| research | GPS raw/fix | 16.278 | 0.2078 | 326 |
| research | IMU | 18.431 | 0.1143 | 369 |

这是SITL复用MAVROS、已有MAVProxy streamrate=25环境的观测值，可能受既有流请求影响，
不代表USB物理链路已测，也不宣称严格达到了请求的5/10Hz。应以实际间隔是否满足接收保护评估。
最终探针stop_exit_code=0；最后只读进程/端口检查无本轮SITL、MAVROS、Gazebo及隔离端口残留。

探针代码：`evidence/v0-research-20261103/readonly_probe.py`，只用经身份核验的本机新SITL实例，
复用唯一MAVROS；外层生产Launch使用start_mavros=false、enable_control=false、dry_run=true，
导航子进程环境PYTHONNOUSERSITE=1，不调用ARM/起降/切模式。分别加载level与research生产YAML，不重写其航点。

先前两次探针失败均保留：

- `20261103T135113-readonly-streams`：探针在MAVROS参数就绪前读取fcu_url而KeyError，未启动导航。
  初次stop因未连接拒绝；随后模拟器/代理已退出，仅残留有精确runtime路径的ROS/Gazebo孤儿进程。
  验证PID/用户/命令行及模拟器/代理不存在后，PID定点停止残留并等待端口释放，清理记录单独保留。
- `20261103T140730-readonly-streams`：探针过早读取stdout缓冲，误报EXTENDED_STATUS日志缺失。
  正常退出后其日志中五组请求全部存在。改用/rosout_agg采集请求日志，并启用探针子进程无缓冲输出；未改生产stream脚本。

## 6. 未测与放行边界

未访问真实机载磁盘，未连接真实FCU或真实机载Master，未ARM/起降/切模式/写参任何真实飞机。
机载现场Launch、level YAML与历史SHA值仅据用户交接，不声称逐字节一致。
负右向YAML与交接的负ENU y历史输出方向不一致，部署前必须从机载原件核对，不改变已验证坐标变换来迎合输出。

仍未完成：真实USB长期遥测频率/丢流恢复、真实UWB质量/重定位、RC接管/飞控failsafe、带动力倾角/围栏触发、
声学采集/自动录包联合验收，以及生产5点科研任务原样实飞。
超速/超时科研策略、倾角/围栏主要由单元与ROS回调测试覆盖，并未逐项追加动态SITL故障注入。
停止发送位置目标不等于停止运动，gate死亡也不能保证物理刹停；不据本记录直接放行真机控制。
