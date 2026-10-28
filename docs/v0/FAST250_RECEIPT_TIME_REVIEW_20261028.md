# V0参考Fast250的接收时间改造与离线验证

日期：2026-10-28（Asia/Shanghai）。基线：`af681a7cedf1e5dc532355aa5d261b11f1948437`，分支 `feat/arducopter-v0-relative-waypoints`。

**SITL继续暂停。本轮只做源码读取、必要修改、单元测试和只读bag分析，没有启动ROS Master、执行rosbag play、启动仿真、访问真实串口或操作无人机。此前失败不改写为PASS。**

## 1. 参考源码、固定版本与对应关系

参考仓库：[ZJU-FAST-Lab/Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)。本次固定commit为 `8ff64274b161bb829653faf5946656658e7e2c9f`；五个文件SHA256保存于 `evidence/receipt-time-20261028/fast250_reference.json`，不以可变master代表永久版本。

以下路径均位于该commit的 `src/realflight_modules/px4ctrl/` 下。

| Fast250源码定位 | 已核实逻辑 | 本项目对应 |
|---|---|---|
| `src/input.cpp:126`，`Odom_Data_t::feed()` | 保存原消息，同时独立记录ROS接收时刻 | `Health.receive()`及manager/gate的pose回调；保留原stamp但不用于接收门槛 |
| `src/input.cpp:167`，`Imu_Data_t::feed()` | IMU也按回调接收时刻计时 | 参考同一原则；V0原本不消费IMU做控制，本次不新增无用途的IMU订阅或安全先决条件 |
| `src/input.h:62` | Odom数据结构含独立接收stamp和原始消息 | `Health.rx_ros`与`header_stamps / pose_stamp`分开保存 |
| `src/PX4CtrlFSM.cpp:509/514/519` | command/odom/IMU使用当前ROS时间减接收时刻判断新鲜度 | `Health.reason()`及gate的`fault()`按接收年龄判断 |
| `config/ctrl_param_fpv.yaml:63` | `msg_timeout`集中组织，odom/cmd/imu等示例为0.5秒 | V0集中四个超时项，缺省0.5秒，state按实际心跳显式覆盖 |
| `src/px4ctrl_node.cpp:26` | 分离state、odom、command、IMU订阅及连接等待 | 保持原有MAVROS订阅、连接/模式检查；不搬PX4接口和控制器 |
| `src/input.cpp:206`，`State_Data_t::feed()` | 原实现仅复制飞控state，没有独立接收计时 | V0按本项目要求额外记录state接收时间；不谎称Fast250原函数已有该功能 |

V0没有移植Fast250的OFFBOARD状态切换、自动起降、姿态/推力输出、100Hz控制器，也没有修改MAVROS内部时间同步。仍为ArduCopter GUIDED位置目标，10Hz样例频率，唯一Control Gate。

固定源码可按以下前缀和表中路径查阅：

```text
https://github.com/ZJU-FAST-Lab/Fast-Drone-250/blob/8ff64274b161bb829653faf5946656658e7e2c9f/src/realflight_modules/px4ctrl/
```

## 2. 实际改造

### 2.1 接收时间与消息头解耦

- manager和gate的MAVROS pose、velocity、state，以及已有GPS/EKF回调记录ROS接收时间。`rospy.get_time()`与`rospy.Time.now().to_sec()`使用同一ROS时钟。
- 超时检查以ROS接收年龄为主，保留已有monotonic接收间隔上界，避免暂停ROS时钟掩盖实际断流。任务超时、周期监测、悬停持续时间仍用monotonic，不依赖原始header推进。
- 重复或回退的header不再导致直接丢弃有效位置样本；检查位置跳变时也不再先因重复header而跳过，避免重复stamp隐藏真正跳点。
- 不改写输入header、不伪造stamp、不创建TimeSyncManager、不加入时钟校准/估计/重建流程。
- manager/gate状态输出增加 `rx_ros_s`、`header_stamps_s`、`stamp_diagnostics`；gate另保留目标接收时刻与原始目标stamp。这里只是最新快照，逐消息诊断仍需记录原始MAVROS pose/velocity话题，不把稀疏快照当全量原始数据。
- 任务目标、状态、心跳的新鲜度统一由gate的command_timeout控制；不再混用0.35与0.5秒硬编码，不再以目标/心跳的原始header年龄作为收包门槛。

### 2.2 仍保留的拒绝与锁存

- pose/velocity必须有限，四元数有效，pose frame仍必须为已审计的`map`；velocity保留该MAVROS插件的`map/base_link`标签兼容，不改变ENU数值。
- ROS时钟未初始化时不接受有效接收状态、不允许start。ROS整体归零或回退清空旧接收、悬停和冻结历史；活动manager任务锁存中止，gate清空旧任务输入并撤销租约。即使时钟在下一次控制循环前已恢复，重置标志也不能被悄悄覆盖。
- MAVROS断开立即清空旧pose/velocity/GPS/EKF状态；连接恢复后必须等新数据。gate对断链和人工离开GUIDED在state回调内就锁存，避免两次回调之间切出再切回被周期检查遗漏；该回调不请求备用模式。
- 清空后未知state中的armed=false不能冒充实际DISARM。即使任务为IDLE，gate也先要求新鲜、connected的FC state；存在故障时不准备或重置租约，避免断链后的默认字段意外清除锁存。
- GPS质量、EKF flags/ratios、原点改变、位姿跳变、姿态、软件边界、持续超速、位姿/速度冻结不一致、唯一发布者、YAML hash和FC参数检查没有删除。
- 接收新鲜不等于定位物理正确。上游缓存持续重放、自洽的错误位置/速度、定位缓慢漂移等仍可能漏检；不能靠本次改造证明UWB真实定位可靠。

## 3. 集中配置与兼容规则

```yaml
msg_timeout:
  pose_timeout: 0.5       # 秒；缺省0.5；从回调接收计时，不是header年龄
  velocity_timeout: 0.5   # 秒；缺省0.5
  state_timeout: 3.0      # 秒；代码缺省0.5，样例显式3.0以兼容约1Hz心跳
  command_timeout: 0.5    # 秒；缺省0.5；gate收到的目标、任务状态及心跳
```

不是把Fast250的0.5秒机械用于所有MAVROS流；FC state常由低频心跳驱动，0.5秒若小于实际消息周期会误报。部署前按真实频率核对。GPS/EKF原有3秒超时保留，不扩展出新的参数框架。

旧 `safety.pose_timeout_s / velocity_timeout_s / state_timeout_s / command_timeout_s` 仍可读。新旧两处同时定义且数值冲突时启动前拒绝，而非静默选择。配置在内部归一化以供旧调用代码复用；重复验证保持一致。

旧 `safety.future_tolerance_s` 仍兼容，但只影响原始stamp诊断，不再放行或拒绝定位输入。新样例去掉了该调参项。缺字段、未知字段、非法值、布尔代数值等schema检查仍在。

本次默认样例的pose接收门槛由0.35改为0.5秒，是公开的配置变化；**下述历史bag对照仍加载历史YAML的0.35秒，不通过改阈值制造对照成功。**

## 4. 三个原始异常：事实与行为差异

只读分析工具：`tools/v0_replay_receipt_time.py`。它从Git读取旧core，加载每例原始mission，在bag记录时刻驱动旧/新pose输入路径；不运行任务、不中途播放消息、不调用服务、不改原证据。最终输出在 `evidence/receipt-time-20261028/historical_bag_comparison_final.json`，含bag/YAML/source SHA256；初版输出也保留，最终归档以final文件的源码哈希为准。

| 原异常与证据 | 本次实际读包发现 | 新行为与不能证明的内容 |
|---|---|---|
| `POSE_FRAME_OR_STAMP`，`20261027T143500-single` | 447个pose全是map。旧gate首次拒绝时header年龄约0.492324s，大于原0.35秒；没有原始stamp回退。该包另有约0.528779s收包间隔 | 不再把header年龄永久锁成pose数据故障，原stamp记OFFSET。真实0.529秒停流若发生在运行中仍应超时；不能把它全部解释成微小同步偏差，也不把历史FAIL改成PASS |
| `POSE_STAMP_REWIND`，`20261027T144135-mavros_loss` | 448个pose全是map；一次原始stamp回退约0.0315418s，当时header年龄约0.072705s；最大bag收包间隔约0.047422s | 在连续接收、有效数据下仅记RAW_STAMP_REWIND，不再拒绝start；只证明输入判定差异，不代表已完成MAVROS掉线注入验收 |
| `POSE_STALE`，`20261026T193632-multi` | 原节点事件记录接收年龄约0.420419s；bag活动阶段最大收包间隔约0.046888s。全包最大间隔约1.095156s、header年龄约1.057171s出现在已ABORT之后，不能用来解释此前中止 | 原节点回调停顿、不同进程调度、队列/CPU等仍需调查。新门槛按真实回调接收计时；旧0.35秒下真实0.420秒节点停流仍会中止，样例0.5也只改变容忍度，不证明调度问题已根治 |

新旧输入路径对照中，前两例旧core分别重现FRAME_OR_STAMP、STAMP_REWIND，新core未出现pose内容fault，且比较配置的pose_timeout都仍为0.35。第三例旧core在中止后的延迟包中也出现header错误，但该现象不是原中止原因；新core不以该header错误额外锁存。

**bag记录时刻不等于原manager/gate回调被调度的时刻。** 此工具没有重建所有飞行状态和无消息间隙的控制tick，不能从 `first_current_pose_fault=null` 推断新任务PASS或真实停流不再存在。原始summary、bag和日志保持字节不变。

## 5. 离线测试与证据

无需ROS Master，source已有ROS消息环境后执行：

```bash
cd /home/liqidun/github/indoor_nav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s tests -p 'test_v0*.py' -v
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s src/indoor_nav/test -v
python3 tools/v0_replay_receipt_time.py
```

`--output <新文件>`以独占创建方式保存结果，不覆盖旧输出。正常执行只读bag需要本地保留的flight.bag；原始大包仍不纳入Git。

覆盖要点：正常接收；原始header小幅回退/重复/偏移/零值；真正pose断流；ROS接收年龄与monotonic间隔独立检查；断链清空与重连必须新数据；坐标系/NaN/四元数/跳点/冻结/EKF；ROS归零/重新初始化；中止后不能续飞；gate命令超时；人工模式切走再切回；采集topic隔离与离线混合消息不刷新导航pose。

真实ROS生成消息用于直接调用回调，rospy时钟和外部接口均mock，不初始化节点、不注册订阅、不访问网络/飞控。采集验证只到静态路由/回调层，不是F3真实ROS高负载联合验收；真实声学程序未改。

证据目录：`docs/v0/evidence/receipt-time-20261028/`。开发中第二次测试曾有一个**测试期望错误**：把103.02相对103.00误写为回退；修正为102.98并去掉导入测试类导致的重复计数，失败日志保留在 `unit_iteration2_failed.txt`。不隐藏该失败，不以重复计数夸大覆盖。

最终测试计数、退出码、命令、源码身份及历史输入保护核验以本目录 `manifest.json` 为准；没有运行catkin重构建、SITL或真机验证。

| 最终源码离线检查 | 结果 | 日志 |
|---|---|---|
| V0专用测试 | PASS，66项 | `v0_unit_tests_final.txt` |
| tests目录全量回归 | PASS，144项（已包含上述66项，不重复相加） | `all_unit_tests_final.txt` |
| 原core测试 | PASS，9项 | `core_unit_tests.txt` |
| Python语法、launch XML、diff检查 | PASS | `manifest.json`记录 |
| 三份历史bag只读对照 | 完成输入路径分析，不是飞行PASS | `historical_bag_comparison_final.json` |
| 当前源码SITL / 真机 | NOT_RUN，保持暂停 / 未授权 | 无新飞行证据 |

## 6. Control Gate约束是否改变

保持唯一MAVROS、唯一gate位置发布者、唯一生产BRAKE请求出口；manager只发任务意图，仍不调用FC模式切换、ARM/起降或参数写入。原点P0/yaw0、ENU位置出口、GUIDED策略及10Hz样例发布频率不变。

默认退出策略仍为manual_takeover；显式健康BRAKE仍仅限已获租约、ARM/GUIDED、独立定位及已核查参数健康且原因在白名单内。ROS时钟重置、断链、frame/定位错误和冲突控制源均不盲目BRAKE。人工离开GUIDED回调立即锁存，不能再抢回模式。

本次没有把停止发布当成刹停，没有改变实机安全放行边界。gate死亡/完全断连时仍依赖FC failsafe和飞手，无法仅靠伴随软件兜底。

## 7. 后续冷启动SITL建议：当前不执行

1. 用户另行解除暂停后，冻结源码与测试工具SHA256，每例独立新runtime/EEPROM，保留原1m起飞、真值Z>0.85m、速度范数<0.06m/s、35秒准备门槛。
2. 首先单点/四点分别冷启动；记录pose原始stamp与ROS接收年龄、实际gate发布间隔、FC mode/速度/真值。比较旧0.35与新0.5配置时分别标记，不混为同门槛复现。
3. 补yaw、高度/返回、持续到达判定；真实停pose流必须RX_TIMEOUT，单独扰动原始header不能造成误拒绝；保留原始header，不在MAVROS内打补丁。
4. 再测manager正常abort/异常退出、gate退出、真实MAVROS死亡/重连、GUIDED切出/切回、ROS时钟重置。确认锁存和不自动恢复；对BRAKE另测实际mode与减速，不只看服务返回。
5. F3先解决原 `after_recorder.bag.active` 断言/生命周期问题，再做唯一MAVROS下的合成采集/录包独立退出和时效压力测试。不能用本次静态/离线回调测试替代真实联合测试。
6. 不重启F2模拟或协议研究。真实UWB/EKF与最新麦克风部署留给另行授权的拆桨检查；本轮没有真机操作资格声明。

当前交付为未提交的源码/配置/测试/文档修改；没有自动commit或push。10月27日暂停报告继续作为历史快照，不回写其Git状态或测试结论。
