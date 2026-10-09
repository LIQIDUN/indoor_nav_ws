# 上一轮历史记录（截至run_04，不代表本轮最终结果）

## 上一轮结果（历史记录）

上一轮完成 **3 次实际起飞，均正常自动降落并解除解锁，当时完整航点飞行尚未通过**。另有1次解锁前被飞控拒绝的尝试，不算起飞。未执行5次重复性或10分钟稳定性验收。

| 原始目录 | 实际飞行 | 起飞/悬停 | 航点 | 降落 | 结果与原因 |
|---|---|---|---|---|---|
| `run_01` | 否 | 未执行 | 未执行 | 保持落地未解锁 | 解锁拒绝：Main loop slow，约304 Hz < 400 Hz |
| `run_02` | 第1次 | 起飞约0.8 m，悬停中止 | 未执行 | 成功、解除解锁 | FC连接仍true，但1 Hz状态消息间隔超过原1秒门禁 |
| `run_03` | 第2次 | 成功 | 启动后立即中止 | 成功、解除解锁 | SETPOINT_TIMEOUT：RUNNING先于首个目标抵达 |
| `run_04` | 第3次 | 成功 | 正常启动，1秒后被工具中止 | 成功、解除解锁 | 新工具错误地给锁存mission_state设置1秒过期时间 |

没有把未执行的方向运动和矩形航点列为通过。第三次的测试工具错误已修复；本轮另获一次复测授权，失败修复后又获明确追加一次授权，结果见下方最新记录。

## 发现及修改

1. **仿真FDM采样不足。** 原Gazebo物理步长2.5 ms，对本机legacy binary backend实际只得到约304 Hz FC主循环。完整ARMING_CHECK揭示此前ARMING_CHECK=0遮住的问题。隔离世界副本改为1 ms后，DataFlash PM.LR实测400 Hz；保留完整解锁检查。
2. **心跳频率与新鲜度门禁不匹配。** run_02中MAVROS connected=true，监督器fc_age_s=1.00198秒，实际状态最大间隔1.00559秒。只对隔离FC请求HEARTBEAT 5 Hz，实测最大间隔约0.205秒；不放宽FC_LOST门禁。
3. **ExternalNav协方差耦合。** 基线VISP.PErr=100，起飞/悬停段EKF水平位置误差RMS0.258 m。启用已有、默认关闭且有固件/三套源参数门禁的SITL位置协方差传输适配，PErr实测0.20 m；run_03同类短段位置误差RMS0.00551 m。此比较不是单变量、同完整轨迹可靠性试验，不能据此给出统计可靠性结论。全局默认仍关闭；EK3_SRC1_VELXY仍0，无UWB速度融合。
4. **任务首目标竞态。** mission_manager.start同步prime合法保持目标后再宣布RUNNING；监督器首次目标也使用已有0.5秒截止时间，重复RUNNING不能续期，等待时不转发目标，超时仍ABORT。持续飞行中的旧目标超时行为不变。run_04未再触发SETPOINT_TIMEOUT。
5. **测试工具锁存话题误判。** mission_state是事件话题，不是周期心跳。改为保留其最新事件，并继续通过周期FC/定位/安全状态及监督器目标超时检查判断活性；ABORT仍立即失败。错误修复已通过回归测试，尚待追加飞行验证。

## 位置与速度对照

统计窗口为TAKEOFF接受到失败事件，不含LAND/地面复位。使用ROS header时间与线性插值Gazebo接收时间对齐；Odometry完整四元数还原ENU，再用实际标定变换逆转到world_lab。没有拟合旋转或选择最佳时间偏移。短时厘米以下结果受Mock零噪声、零延迟影响，不能代表真实UWB精度。

| 实际飞行 | EKF水平位置RMS / 最大(m) | EKF水平速度RMS(m/s) | UWB位置RMS(m) | VISP.PErr(m) |
|---|---|---|---|---|
| 第1次 | 0.25756 / 0.33419 | 0.11559 | 0.00227 | 100 |
| 第2次 | 0.00551 / 0.00925 | 0.01057 | 0.000385 | 0.20 |
| 第3次 | 0.00649 / 0.00894 | 0.01249 | 0.000381 | 0.20 |

第1次UWB派生速度有效56样本，水平误差RMS0.00224 m/s。第2/3次水平近似静止，窗口标记STATIONARY_OR_FROZEN_AMBIGUOUS，未发布有效影子速度，不能计为速度测试通过。影子速度未送飞控。

轨迹、原始遥测、每次events/summary及分析在 `docs/evidence/sitl_flight/run_0*/`。每次含独立ROS bag和DataFlash快照；同一FC进程内DataFlash含整个启动序列，分析用VISP.RTimeUS限定各次窗口，不能把整个文件都算作该次飞行。

![基线飞行](evidence/sitl_flight/run_02/trajectory.png)
![最新飞行](evidence/sitl_flight/run_04/trajectory.png)

XKF3水平位置创新RMS，core0/core1：第1次0.248/0.255 m；第2/3次日志0.01 m量化后为0，不代表连续值严格零。ExternalNav接收有VISP，源选择和“using external nav data”有状态文本，强位置观测后EKF与真值误差明显降低；目前完整动态融合/方向验证仍未完成，不能仅凭PErr下降断言航点闭环通过。

三次飞行定位状态均持续OK；没有POSSIBLE_UWB_FREEZE、FC_IMU_STALE或INVALID_VELOCITY_FRAME。run_03的SETPOINT_TIMEOUT是实际ABORT并触发LAND；run_04是工具主动失败收尾，监督器没有ABORT。地面/起降阶段出现Z_BELOW_FENCE是原有高度门禁的预期表现，未将其关闭。

## 工具与隔离

新增 `tools/sitl_flight_instance.sh`、`tools/indoor_nav_flight_test.py`、`tools/analyze_sitl_flight.py` 和 `config/mission_sitl_flight.yaml`。SITL launch新增可选mission_file，原默认航线不变。

使用独立ROS Master11321、Gazebo Master11355、MAVROS loopback14572，不启动QGC。每次控制动作前核对实际SITL二进制/cwd/PID/创建时间、MAVProxy唯一TCP主链路及唯一UDP输出、MAVROS实际进程归属、loopback UDP绑定、命令服务同一提供者、FC版本/系统号/SIM参数。身份不符即拒绝。在线EKF监控解码ArduPilot实际发送的MAVLink EKF_STATUS_REPORT(193)，检查姿态、水平/垂直速度、相对水平/垂直位置有效及初始化状态；不依赖可能没有消息的ESTIMATOR_STATUS(230) ROS话题。run_04 bag中193消息370条、flags均831。

参数写入只在隔离EEPROM副本；每次冷启动重新复制原EEPROM。原工作目录和实机配置未修改。

人工门禁来自 `docs/SITL_DEBUG_AND_FLIGHT.md`；操作员本轮明确确认仿真origin并授权最多3次实际飞行的正式ACK。工具不自动伪造ACK或READY，fly只消费真实READY。origin为启动器明确指定的仿真home，并有接收回读；不是GPS测量。

复现流程（需在有主机ROS权限的终端、确认无端口冲突后执行）：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 持久用户服务；adapter依旧必须显式opt-in，仅隔离SITL。
systemd-run --user --unit=indoor-nav-autoflight --collect \
  --setenv=SITL_FLIGHT_COVARIANCE_ADAPTER=true \
  --working-directory="$PWD" "$PWD/tools/sitl_flight_instance.sh"
export ROS_MASTER_URI=http://127.0.0.1:11321
python3 tools/indoor_nav_flight_test.py prepare --output docs/evidence/sitl_flight/prepare_NEW
# 人工核实prepare结果、origin和落地未解锁后，正式确认：
rosservice call /indoor_nav/safety/acknowledge
python3 tools/indoor_nav_flight_test.py fly --output docs/evidence/sitl_flight/run_NEW
python3 tools/analyze_sitl_flight.py docs/evidence/sitl_flight/run_NEW
```

不自动无限重试。fly失败也尝试身份复核后的LAND，只有真值落地且FC未解锁才reset和恢复Gazebo初始地面姿态；连接/身份丢失时不冒险发命令，记录RECOVERY_FAILED并保留现场。SIGINT/SIGTERM进入清理路径。每次reset重新要求正式ACK。原始bag/BIN/tlog受仓库ignore规则保护，仍留存在本地交付目录。

## 测试、回滚、下一步

71项主测试、9项核心测试PASS，无skip；catkin_make、compileall、shell语法和git diff --check通过，见 evidence/sitl_flight/validation.txt。覆盖首次目标截止、重复RUNNING不能续期、无目标不转发、锁存事件不假报过期、周期FC陈旧和EKF无效仍拒绝及错误Master/实机URL拒绝。另有原冻结、坐标、协方差、时间戳回归。

回滚仅撤销本次mission启动prime、supervisor首次目标截止及新增测试工具/可选SITL航线；保留此前所有未提交修改，不执行reset/clean。隔离FC仅改ARMING_CHECK=1、LOG_DISARMED=1，原副本初始分别0/0；完整停掉隔离服务后重新复制原EEPROM即可回滚副本。HEARTBEAT间隔冷启动复原，物理步长仅在隔离世界副本，适配默认仍关闭。

最新状态：本轮已完成完整航线及±X/±Y方向验证。下一步仅建议改进移动位置目标的跟踪滞后，再逐步加入Mock噪声/延迟；不开展重复性或长时间可靠性验收。
