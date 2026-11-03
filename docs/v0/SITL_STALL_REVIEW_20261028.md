# 2026-10-28 MAVLink/Pose停流：轻量根因排查

## 结论

**更可能是SITL ↔ Gazebo仿真推进/数据交换的瞬时停顿，而不是MAVROS原始时间戳误判或导航回调单独卡住。GUI不是必要条件，也没有证据证明主机CPU饱和是主因。**

本轮只补一组独立冷启动：GUI开启PASS，无GUI在较低主机CPU占用下再次FAIL。因而**不能按“GUI偶发负载、已关闭”结案**；仍保留“未解决的仿真链路稳定性问题”。不为此修改生产导航代码，不延长0.5秒pose超时。若需要稳定、可重复的SITL验收，后续应窄范围检查SITL/Gazebo数据交换与线程等待；本轮不继续追加实验，也不调查真机链路。

## 版本、范围与约束

- 分支：`feat/arducopter-v0-relative-waypoints`；本轮基线HEAD：`dcf3919d6b14d5531e4044c892bc4572e8875ffd`。没有提交、改写历史或改动生产代码；只新增本报告及本轮证据。原四份未跟踪用户输入未操作。
- 历史两例输入记录的HEAD为`255f49589063067a2e1d77f038beadce914a9cbd`；新两例为上述`dcf3919...`。实际生产模块、manager、Gate、launch、参数文件及飞行驱动/SH的SHA256一致，见`integrity.json`。新运行的dirty状态来自新增排查材料，不代表修改了飞行代码。
- 复用`tools/v0_sitl_suite.py`和`tools/v0_sitl_instance.sh`，内置GPS、Copter-4.5.7、同一iris world；每例独立runtime/EEPROM，串行运行。
- 保持pose超时0.5秒、`WPNAV_SPEED=20 cm/s`、1 m起飞目标、真值Z>0.85 m、速度范数<0.06 m/s、35秒准备期限。新旧yaw YAML除`mission_id`外语义一致。
- 没有操作真实串口或飞控，没有改变ExternalNav、MAVROS、声学业务或安全阈值。

## 对齐结果

以下间隔为MAVProxy tlog中FCU `LOCAL_POSITION_NED`的墙钟接收间隔；bag中Pose间隔与之吻合。时间均为Asia/Shanghai。

|证据目录末段|GUI|结果|最大位置间隔|该间隔FC boot时间增量|
|---|---|---|---:|---:|
|`20261028T143245-yaw`|开|原FAIL|547.136 ms|40 ms|
|`20261028T143753-mode_exit`|开|原FAIL|1616.888 ms|40 ms|
|`20261028T150340-yaw`|开|历史补测PASS|60.444 ms|40 ms|
|`20261028T150619-mode_exit`|开|历史补测PASS|47.980 ms|40 ms|
|`20261028T180543-yaw`|开|本轮PASS|47.436 ms|40 ms|
|`20261028T180821-yaw`|关|本轮FAIL|546.308 ms|40 ms|

**纠正范围而非改写失败：**14:33:43.654–14:33:44.201是约547 ms位置停流；14:38:45.814–14:38:47.431的完整停流实际约1.617秒，不是也只有540 ms。原0.5秒保护是在完整停流结束前触发，不能把触发时已等待的时长当成总停流时长。两次原FAIL仍为FAIL，且都在HOLD_INITIAL中止，原mode_exit尚未执行人工切模式注入。

### 为什么更像上游仿真停顿

1. 历史两例FCU源`system=1, component=1`全消息分别停了540.548 ms和1604.048 ms；bag `/mavlink/from`分别停约540.656 ms和1604.206 ms。因此不是只有Pose插件或导航线程停顿。
2. 同一停流期间，tlog仍记录MAVROS源`1:240`的TIMESYNC/位置目标，以及MAVProxy源`245:190`的消息；历史两例各有5条MAVROS位置目标进入tlog。说明MAVROS→MAVProxy方向及其主循环并未整体冻结。**这不等于飞控已执行这些命令。**
3. FCU在停流前后MAVLink序列号连续（模256差为1）；位置消息`time_boot_ms`仅增加40 ms，墙钟却多出约0.55/1.62秒。恢复0.5–2.5秒后的墙钟减boot时间偏移仍分别增加约0.405/1.491秒，不像单纯MAVROS下游短时缓存后立即追平。这里比较的是增量，不把boot时间当UTC。
4. 历史bag中Gate/watchdog最大间隔仍约100.25/101.51 ms；原始pose header无回退，frame均为`map`。不是`POSE_STAMP_REWIND`重现，而是真实接收断流被`POSE_RX_TIMEOUT`捕获。
5. 本轮无GUI复现进一步提供同步Gazebo证据，见下一节。因此优先级为：**SITL/Gazebo推进或FDM交换端 > SITL遥测输出/MAVProxy上游接收 > MAVROS下游接收/导航时间戳逻辑**。没有直接抓取FDM UDP/TCP，不能完全排除上游交换/输出阻塞，更未定位到具体线程或代码行。

### 无GUI复现的同步负载证据

- 位置停流：18:09:19.644456–18:09:20.190764，546.308 ms；FCU全消息间隔539.668 ms，FC boot仅前进40 ms。
- Gazebo统计相邻样本：18:09:19.658457 → 18:09:20.181728。仿真时间`64.331 → 64.351 s`，Gazebo真实时间`64.283660 → 64.806888 s`；即523.228 ms内仿真仅推进20 ms，区间实时率约**0.038**。两条统计均`paused=F`，不是主动暂停world。
- `gz stats`显示的平滑实时率此时仍为1.00→0.99，**不能只看这个平滑值忽略区间停顿**。分析使用相邻simtime/realtime差分，且排除了DISARM后的清理阶段。
- 停流前后±0.5秒采样窗口内，主机总CPU约14.1–17.4%；SITL峰值约8.0%、MAVProxy15.9%、MAVROS8.0%、gzserver23.9%（进程100%表示占满一个核）。CPU pressure avg10约0.13–0.16%。没有观察到CPU饱和或主线程数百毫秒runqueue等待；只采了主线程schedstat，**不能据此排除某个工作线程/GPU/短时调度问题**。
- GUI开启对照没有超时，位置最大间隔47.436 ms，飞行观测期最小区间实时率约0.975。GUI正常重复能够通过，但关闭GUI也会失败；本轮两例不足以估计发生率，不能声称“无GUI频繁发生”或“GUI因果已经证实”。

## 旧证据能与不能说明什么

- 旧suite记录逐例stop成功及端口释放；yaw与mode_exit分别在各自冷启动约66秒、60秒后出现停流，不与下一实例启动重叠。没有支持“双SITL实例撞端口”的证据，但旧记录不是全主机任务审计。
- 原两例没有有效的同时刻Gazebo实时率/逐进程CPU记录。`gui_process_audit.txt`是14:40:49的快照，不能拿来证明14:33/14:38负载。
- 原`gazebo_stats*stderr.txt`返回“无运行实例”，不能作为实时率证据。失败summary也没有保存逐样本CPU；不从后续成功轮次倒推旧故障CPU。
- 只读检查旧Gazebo/SITL/MAVProxy/MAVROS日志：没有与停流对齐且能独立确证根因的报错。退出后的connection refused、GUI SIGTERM升级与MAVProxy日志线程退出异常不作为飞行中停流原因。
- 只读解码原DataFlash：邻近PM记录的模拟循环率400 Hz、MaxT约2998 µs、NLon=0，DSF Dp=0；这些是仿真时间域数据，不足以排除墙钟暂停，也不是主机CPU测量。
- 本地Copter-4.5.7源码的`SIM_Gazebo.cpp::recv_fdm()`阻塞等待FDM，仿真时间取自FDM时间戳；Gazebo插件的`OnUpdate()`中收电机命令再发状态，在线接收有最长1000 ms等待。这说明**相互等待/交换瞬停是合理候选**，不是证明本次一定丢了某个UDP包。详见`source_probe.txt`。

## 证据、复现及结束状态

新增索引：`docs/v0/evidence/stall-review-20261028/manifest.json`；摘要：`timing_analysis.json`、`resource_analysis.json`、`integrity.json`。原bag/BIN/tlog和旧结果均不覆盖。二进制仍在本地原路径，manifest列出SHA256，不加入Git。

本轮观测入口（本任务已执行完毕，不代表应继续追加）：

```bash
cd /home/liqidun/github/indoor_nav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 docs/v0/evidence/stall-review-20261028/observe.py
```

该入口只串行跑GUI=true/false各一例原yaw suite，附加只读约4 Hz进程观测和Gazebo统计。不改生产节点；若suite非PASS即不再追加实例。离线重分析：`python3 docs/v0/evidence/stall-review-20261028/analyze.py`。

两例均完成LAND/DISARM、stop退出码0及端口释放；无GUI suite返回1表示飞行FAIL，不是清理失败。最终主机审计确认无本轮ROS/SITL/Gazebo残留；原MAVROS PID2391064/3263919仍在，未操作。见`final_host_audit.txt`。

**是否继续修复：**无需为此改导航超时或通信架构；但仿真稳定性尚未关闭，不能把新FAIL改称PASS或据此宣布真机可靠。若继续工作，范围应限于SITL↔Gazebo交换/等待的定点诊断，而不是MAVROS时间同步重写。本轮到此停止。
