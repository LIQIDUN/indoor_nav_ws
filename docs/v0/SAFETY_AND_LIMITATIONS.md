# 安全边界与尚不能证明的结论

- 本交付是软件与仿真证据，不是物理安全认证。真实 UWB/Fake GPS、机载完整采集、RC备用模式和实体围挡未验证；F2未通过，禁止建议自动航点实飞。
- 生产代码不 ARM/起飞/降落、不写 FC 参数、不请求任何模式；只消费现有单一 MAVROS。人工切出 GUIDED 后锁存，绝不重进；只有明确 reset（DISARM）与 start 才能再授权。
- abort、watchdog故障、进程退出统一停发，不声称安全悬停。Copter位置目标可持续追踪最后点；GUID_TIMEOUT不能假定会取消位置目标。定位坏时没有可靠的“当前点”。需要预验证的 RC/FCU failsafe；室内不默认 RTL。
- watchdog不拥有 setpoint，不双写；只能在 manager还活着且ROS图/通信有效时撤销其许可。manager卡死/被kill、Master/MAVROS死亡、CPU全阻塞时，它不能保证任何FC动作。没有硬实时、没有物理急停。
- 单发布者检查为周期ROS图观测，存在检测窗口，不是原子安全锁或防恶意隔离；不要同时启动旧 supervisor 或其它控制插件输入。
- 软件边界相对 P0，不是房间地图，不含旋翼/机身碰撞模型；初始化高度不是 AGL。真实起点安全余量必须人工测量。
- pose跳点/时间异常会中止，绝不重捕获原点续飞；小幅、缓慢或同源一致性重定位仍可能漏检。Fake GPS必须补真实adapter reset/freeze信息，不能只依赖EKF输出。
- 零位移不是冻结。持续位姿/速度不一致检测仅覆盖部分故障；EKF位置与速度一起冻结或错误但自洽可漏检；上游Bool不等于完整测量质量认证。
- 时间期限采用 monotonic；ROS pause/rewind、future/stale stamp 失效。仿真启动/时间同步收敛前的异常同样 fail-closed，需明确 DISARM/reset再检查，不放宽门槛。
- 目前只审计Copter4.5.7/MAVROS1.20.1；新参数WP_SPD、其它固件、其它frame插件不自动兼容。FC参数只读周期检查，不阻止地面站/飞手中途更改；短检测窗口仍存在。
- F3 mock只等价于消息大小和通信负载，不能证明 ADC采样连续性、USB稳定性、24bit格式、增益、真实ESC或ARM录包触发。真实机载R4必需。
- 进程/模式/速度/真值测试逐项查看 SITL_REPORT，不把未测行为标为通过。所有失败尝试保留；历史run_06不计入本轮成功次数。
