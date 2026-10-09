# UWB 水平速度能力审计

2026-10-09，基线 c5baa43，分支 master；开始时工作树干净，前轮修复已由用户纳入基线。本次不提交、不重启现有会话、不写飞控参数。

| 链路 | 证据 | 结论 |
|---|---|---|
| ROS Nodeframe2 | `src/nlink_parser/msg/LinktrackNodeframe2.msg:8` 定义 vel_3d[3] | 字段存在，不等于实际设备可靠 |
| 线协议解析 | `src/nlink_parser/src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c:24,69` 从 int24 解析 vel_3d | 不是 ROS 驱动根据位置差分伪造 |
| 标度 | `nlink_utils.h:68` MULTIPLY_VEL=10000，位置除以1000、EOP除以100 | 证明数值缩放，物理单位/坐标系仍须具体设备协议佐证 |
| ROS 填充 | `src/nlink_parser/src/linktrack/init.cpp:209` ARRAY_ASSIGN | 真实将协议速度拷贝入 ROS 消息 |
| 设备型号/固件、速度质量位、内部算法 | 仓库没有对应实物确认记录 | 未确认，不宣称所有 LinkTrack 都支持可靠原生速度 |
| Mock | `mock_uwb_node.py` vel_3d 固定零 | 明确不模拟已验证原生速度，也不用真值速度冒充 UWB 测量 |

local_time/system_time 均为 uint32，ROS 消息无 Header。设备时钟的单位、回绕/重启与同步条件必须核对具体协议；不能直接转换为 ROS 绝对时间。默认用 ROS 接收时间拟合，非零设备计数器仅用于重复/乱序检测，不声称已同步。全零计数器无法可靠去重，必须诊断此限制；位置不变不是重复包判据。

实现策略：默认 DERIVED_WINDOW，影子诊断，不进入飞控速度观测。只有提供具体设备验证记录，明确 m/s、uwb_anchor 坐标、Tag 速度及有效性之后，才允许配置 native_verified。SITL 强制不选 Mock 原生速度。原生与派生候选均独立展示；原生不可用回退拟合，不将字段固定零当作可靠速度。

安装杆臂：派生速度拟合前先把每个 Tag 位置减去当时姿态旋转的杆臂；原生 Tag 速度减去 R(q)(ω×r)。world→nav 只旋转、不平移。位置与派生速度使用同一输入，统计相关，禁止未经独立评估同时高置信度融合。

状态：驱动能力审计已完成；实物型号、固件和协议语义证据不足；默认不启用原生速度，不改协议。
