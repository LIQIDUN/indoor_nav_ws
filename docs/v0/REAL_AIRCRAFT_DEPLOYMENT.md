# ArduCopter V0 实机部署操作手册（待人工审阅）

核对日期：2026-10-28。源码基线：`feat/arducopter-v0-relative-waypoints` / `dcf3919d6b14d5531e4044c892bc4572e8875ffd`。

**本文是供后续人工批准使用的操作手册，不是实机放行记录。本轮仅编写文档，未执行下列安装、ROS启动、飞控连接或飞行步骤。所有“预期”均为源码规定，不代表已经在机载实物上观察到。** 完成本文审阅后，再按 [人工审阅清单](MANUAL_REVIEW_CHECKLIST.md) 签核；带桨阶段还需 [分级SOP](REAL_AIRCRAFT_SOP.md) 的场地与飞手批准。SITL基础设施停顿作为已知限制归档，本轮不再调查，不删除其FAIL。

## 1. 系统与职责

真实定位链路为：**UWB设备 → Pixhawk的GPS接口 → ArduCopter EKF3 → MAVROS**。这一实物接入方式由用户确认；其精度、延迟、坐标方向、失效行为尚需拆桨核对。

机载只有一个ROS Master、一个MAVROS和一个FCU通信口占有者：

- 采集业务：现有XVF3800、六路IM72D128、振动、ESC/IMU及ARM/DISARM自动rosbag。
- 导航业务：`v0_nav_manager`读取YAML，生成`/v0_nav/target_pose`任务意图；`v0_nav_watchdog`即Control Gate，独占`/mavros/setpoint_position/local`。
- 两业务不管理彼此进程。导航不打开UWB/飞控串口，不启动MAVROS、roscore、传感器或rosbag。
- 使用**ArduCopter GUIDED位置目标**，不是PX4 OFFBOARD；Fast250仅为通信接收计时的参考，**不安装、不运行Fast250控制器**。不需要pymavlink、DroneKit、旧ExternalNav/world_estimator或Fake GPS模拟器。

## 2. 先填写部署记录，不套用开发机地址

|项目|已知事实 / 需填写内容|
|---|---|
|开发机工作区|`/home/liqidun/github/indoor_nav_ws`；不是机载电脑路径承诺|
|机载导航工作区`NAV_WS`|**实机待核对**：绝对路径，不能覆盖采集工作区|
|采集工作区`ACQ_WS`|**实机待核对**：机载最新版，不以本地旧副本代替|
|ROS Master|**实机待核对**：现有唯一Master的地址和端口|
|ROS_IP / ROS_HOSTNAME|**实机待核对**：本机及其它ROS机器互相可达的地址；只选一种|
|FCU设备及波特率|**实机待核对**：实际USB/UART连接；不是固定`/dev/ttyACM0`|
|飞控固件 / system / component|代码目前限定Copter **4.5.7**；EKF原始报文筛选为 **1 / 1**|
|采集入口 / 保存位置|**实机待核对**：launch绝对路径、recorder版本、磁盘挂载及空闲空间|
|实验编号|每轮唯一，关联YAML、导航日志、声学bag及飞控BIN|

下列变量是**填写模板**，所有终端使用同一组值；不要原样使用占位路径：

```bash
export NAV_WS="/absolute/path/to/indoor_nav_ws"
export ACQ_WS="/absolute/path/to/current_sensor_workspace"
export MISSION_FILE="/absolute/path/to/reviewed_single.yaml"
export EXPERIMENT_ID="R0_日期_架次"
export SESSION_DIR="$HOME/flight_evidence/$EXPERIMENT_ID"
```

`SESSION_DIR`须选不存在的新目录，保护已有实验：`mkdir -p "$HOME/flight_evidence"; mkdir "$SESSION_DIR"`。跨终端重新设置这些变量，或source人工审阅过的本轮环境文件；不自动改写`.bashrc`。

## 3. 软件环境与构建

### 3.1 已验证的开发环境，不等于机载环境

本机只读核对：Ubuntu **20.04.6**、Python **3.8.10**、ROS1 **Noetic**；MAVROS及mavros_msgs **1.20.1**、catkin **0.8.12**、PyYAML **5.3.1**。机载处理器架构、系统、包版本均待核对。ROS Noetic已于2025-05-31结束常规支持，不能把新部署当作仍有常规维护的全新平台；本轮不升级或迁移ROS。[O1]

在机载电脑上人工检查（任何工作目录均可）：

```bash
cat /etc/os-release
uname -m
python3 --version
source /opt/ros/noetic/setup.bash
rosversion -d
rosversion mavros
```

预期为经批准的Noetic环境，MAVROS兼容本项目已审计接口。若没有Noetic或固件并非4.5.7，**先停止部署并登记差异**，不要改schema/版本门槛绕过。

### 3.2 安装依赖：仅在维护者批准的机载维护窗口

生产V0直接需要PyYAML和ROS包；`psutil`用于部分检查/测试工具，不是飞控直连库。声学驱动另有自己的依赖，沿用机载已验证环境，不在此重新安装/升级采样栈。

```bash
# 工作目录不限；前提是已配置经过维护者确认的ROS Noetic软件源
sudo apt-get update
sudo apt-get install build-essential python3-yaml python3-catkin-pkg python3-rosdep \
  ros-noetic-catkin ros-noetic-rospy ros-noetic-roscpp ros-noetic-rosgraph \
  ros-noetic-std-msgs ros-noetic-std-srvs ros-noetic-geometry-msgs \
  ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-message-generation \
  ros-noetic-message-runtime ros-noetic-tf2-ros ros-noetic-geographic-msgs \
  ros-noetic-rosgraph-msgs ros-noetic-gazebo-msgs \
  ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-rosbag
# 仅需运行依赖psutil的工具时：sudo apt-get install python3-psutil
```

仓库`package.xml`仍含旧模式的`nlink_parser`构建依赖，完整源码应含`src/nlink_parser/extern/serial`；V0 launch不启动它，不因此再接一个UWB串口。`gazebo_msgs`是现有包的消息依赖，不代表实机应启动Gazebo。不要用系统级`pip install --upgrade`替换已验证ROS/Python环境。

MAVROS需要GeographicLib数据；先查`ls /usr/share/GeographicLib/geoids/`及现有MAVROS部署记录。若确实缺失，再由维护者按官方ROS1安装说明执行数据安装；本机1.20.1安装器位于`/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh`，需网络和系统写权限，**不要每次启动重复安装**。[O2]

```bash
# 仅缺失数据且路径已核实后，工作目录不限：
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 3.3 部署源码与构建

由维护者提供**人工批准的仓库或源码快照**。此文档尚未提交；只clone现有HEAD不会自动得到未提交手册/证据，后续人工归档后再确定交付SHA。不得覆盖原采集工作区，不复制开发机`build/`、`devel/`、EEPROM、SITL runtime或bag。

若采用Git交付，以下只用于**尚不存在的目标目录**，`APPROVED_REPO`和`APPROVED_COMMIT`均由维护者填写：

```bash
export APPROVED_REPO="/path/to/approved/repository_or_bundle"
export APPROVED_COMMIT="人工批准的完整40位commit"
test ! -e "$NAV_WS" || { echo "目标已存在，停止，先审查/备份"; exit 1; }
git clone --single-branch --branch feat/arducopter-v0-relative-waypoints \
  "$APPROVED_REPO" "$NAV_WS"
test "$(git -C "$NAV_WS" rev-parse HEAD)" = "$APPROVED_COMMIT" || exit 1
```

若目标已存在，先`git -C "$NAV_WS" status --short`和记录HEAD，交维护者决定更新方法；不`reset --hard`、不清理或stash用户文件。

构建终端，工作目录明确为`NAV_WS`：

```bash
source /opt/ros/noetic/setup.bash
# 仅当采集工作区已经存在且已审阅，作为underlay：
source "$ACQ_WS/devel/setup.bash"
cd "$NAV_WS"
test -d src/nlink_parser/extern/serial || exit 1
# 若rosdep已初始化，可先只检查：
rosdep check --from-paths src --ignore-src --rosdistro noetic
# 缺依赖时由维护者确认rosdep清单，再另行安装；不忽略错误继续
catkin_make -j2
source "$NAV_WS/devel/setup.bash"
rospack find indoor_nav
rospack find rs485_driver_sound
python3 -c 'import rospy, rosgraph, yaml; from indoor_nav.v0 import load_config; print("imports OK")'
```

预期构建结束无错误，`rospack find`指向批准的导航包和**当前机载采集包**。本包使用`catkin_python_setup()`及工作区相对路径记录源码SHA，建议按上述源码/devel方式部署，不假设`install/`或只复制单个脚本也可运行。不要无核查地反向overlay，避免同名包遮蔽。

## 4. 实物连接与上电：先拆桨

1. **本轮不执行。** 后续R0必须拆桨，隔离动力危险，固定机体；维护者确认供电、电压、接地、USB回供和接插件定义。无厂商/既有接线记录时不猜GPS针脚或电平。
2. UWB定位设备保持接入Pixhawk GPS口，导航只读EKF融合结果。机载到Pixhawk的MAVROS连接是另一条已核对的USB/UART通道，不能混为同一GPS口。
3. XVF3800、IM72D128六通道、振动/RS485适配器按机载最新版接线表连接。不要因USB枚举变化改采样率、位宽、通道、增益或分块。
4. 按现有实物SOP先检查遥控器电量、模式开关和失联行为，再依已核实顺序上电机载、定位与飞控；等待UWB与EKF稳定。具体电源顺序由实物负责人签核，不以本文替代硬件手册。
5. 仅枚举设备：`lsusb`、`ls -l /dev/serial/by-id/`、`arecord -l`；对选定路径执行`readlink -f "$FCU_DEVICE"`、`udevadm info --query=property --name="$FCU_DEVICE"`。`FCU_DEVICE`是人工填写的设备路径，不是MAVROS整条URL。
6. 检查`id -nG`、`ls -l "$FCU_DEVICE"`和`lsof "$FCU_DEVICE"`。串口所属组常为dialout，音频权限取决于实际设备规则；若确需`sudo usermod -aG dialout "$USER"`，需重新登录生效。不要`chmod 777`、用root运行全部ROS或再开串口终端抢FCU。

**拆桨阶段全程禁止ARM/起飞。** 本手册不提供实机自动ARM/起飞/降落命令。

## 5. ROS启动顺序与唯一连接

### 5.1 每个ROS终端先加载环境

以下终端均在机载同一用户环境使用，source顺序一致：

```bash
source /opt/ros/noetic/setup.bash
source "$ACQ_WS/devel/setup.bash"
source "$NAV_WS/devel/setup.bash"
cd "$NAV_WS"
```

随后填写**现有**Master地址。在所有ROS进程只位于同一机载电脑、无需其它ROS机器接入的情形，可经确认使用：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
```

有多机ROS时，把Master地址及`ROS_IP`换成本机实际可达LAN地址；或使用经双向解析核实的`ROS_HOSTNAME`并unset `ROS_IP`。SSH进机载运行终端不自动要求多机ROS。**不要复制SITL的11421/14672路由。**

### 5.2 终端M：已有Master优先

```bash
echo "$ROS_MASTER_URI"; echo "ROS_IP=${ROS_IP:-} ROS_HOSTNAME=${ROS_HOSTNAME:-}"
rosnode list
pgrep -af 'rosmaster|mavros_node'
```

能列出预期现有节点就复用，不再`roscore`。若维护者确认尚无Master且机载是Master主机，才在终端M运行`roscore -p 11311`（端口按部署记录替换）。无法连接不等于不存在：先查地址、端口、网络及进程，不直接新开第二份。

### 5.3 终端A：审核采集入口，再启动一次

本机找到两份**仅供比对**的历史副本：

- `/home/liqidun/github/onboard_ros_sensor/ros_sensor/src/rs485_driver_sound/launch/all_sensors.launch`：含振动、XVF3800、**一次`mavros/apm.launch`**及自动recorder；未找到IM72D128节点。
- `/home/liqidun/github/ros_sensor/src/rs485_driver_sound/launch/all_sensors.launch`：只有振动节点，没有MAVROS。

因此名字相同不代表内容相同，不能用第二份判断“采集从不启动MAVROS”，也不能把第一份覆盖机载最新版。

在机载核对实际解析到的文件及其include：

```bash
rospack find rs485_driver_sound
sed -n '1,220p' "$(rospack find rs485_driver_sound)/launch/all_sensors.launch"
# 根据include继续只读核查；记录文件SHA
sha256sum "$(rospack find rs485_driver_sound)/launch/all_sensors.launch"
```

**若人工确认机载入口与上述onboard副本的参数接口一致**，可使用以下命令模板；该副本确有`fcu_url`、`save_dir`、`bag_prefix`三个arg：

```bash
# 工作目录为NAV_WS；FCU_URL/BAG_DIR由部署记录填写，不用历史默认串口
export FCU_URL="/dev/serial/by-id/实际飞控设备:经核实波特率"
export BAG_DIR="/absolute/path/to/acquisition_bags"
export BAG_PREFIX="$EXPERIMENT_ID"
roslaunch rs485_driver_sound all_sensors.launch \
  fcu_url:="$FCU_URL" save_dir:="$BAG_DIR" bag_prefix:="$BAG_PREFIX"
```

接口不同或IM72入口另有文件时，**停止使用此模板**，将实际入口/参数填入审阅清单。采集已经运行就不要重复启动。

若最新版采集入口**经核实不启动MAVROS**且当前没有MAVROS，才由指定连接终端启动唯一一份：`roslaunch mavros apm.launch fcu_url:="$FCU_URL" gcs_url:="" tgt_system:=1 tgt_component:=1`。若已有MAVROS，绝不能执行该命令。单独MAVROS的生命周期不得再由另一采集launch重复管理。

### 5.4 终端I：连接检查，尚未启动导航

```bash
rosnode list
rosnode info /mavros
rosparam get /mavros/fcu_url
timeout 5 rostopic echo -n 1 /mavros/state
rosparam get /use_sim_time
rostopic info /mavros/local_position/pose
rostopic type /mavros/local_position/velocity_local
```

预期`connected: True`、拆桨阶段`armed: False`、实际模式符合飞手设置；速度类型为`geometry_msgs/TwistStamped`。`/use_sim_time`应为false，未设置时ROS默认为墙钟；若为true，停止并查残留配置，**不在活动业务上随意改时钟参数**。`connected=False`、错Master/设备或多实例时不得继续。

`rosnode`的同名覆盖可能隐藏重复启动：还需`pgrep -af mavros_node`、对真实设备`lsof "$FCU_DEVICE"`核对PID与Master。不杀无关实例；发现争用先由维护者处理。物理FCU口应只有批准的一个MAVROS占用者。

## 6. 选择YAML与只读联调

### 6.1 先审阅任务，不直接飞默认方形

默认`config/mission_v0.yaml`有**5点**（含起点停留和返回），不等于SITL驱动生成的4点用例。首次带桨短点只用经审阅的独立YAML，建议人工复制模板到实验配置目录，修改`mission_id`及`waypoints`为单个0.2–0.3 m前向、`z=0`；本轮不代替用户制作/批准实飞配置。

`x/y/z`单位m，分别是**任务启动时**前/右/上，相对唯一P0/yaw0，不相对上一点。`z=0`保持起始EKF Z，不是地面高度。默认固定初始yaw；每点不支持额外yaw字段。

离线检查不连接ROS（工作目录`NAV_WS`）：

```bash
python3 tools/test_v0_relative_waypoints.py --config "$MISSION_FILE"
sha256sum "$MISSION_FILE"
```

预期JSON含`result: PASS`及正确点数、YAML hash和yaw=0的示例目标；**这不是飞行放行，也不检查实机参数**。未知字段、重复ID、越界、非有限数值会报错，按速查表修正，不削弱安全门槛。

### 6.2 终端N：只读导航

```bash
roslaunch indoor_nav indoor_nav_v0.launch \
  mission_file:="$MISSION_FILE" evidence_dir:="$SESSION_DIR/v0_nav" \
  enable_control:=false dry_run:=true
```

此launch只有manager和Gate，不include MAVROS。另一个已加载同样环境的检查终端：

```bash
rostopic echo -n 1 /v0_nav/status
rostopic echo -n 1 /v0_nav/watchdog
rostopic info /mavros/setpoint_position/local
python3 tools/test_v0_relative_waypoints.py --config "$MISSION_FILE" --ros-smoke
```

预期manager `state=IDLE`、`control_enabled=false`；V0不注册FCU目标publisher。smoke的`PASS`只核对只读模式/图及收到消息，不独立证明GPS/EKF合格或飞机可飞。`health=DISARMED`在遥测新鲜的地面阶段可以正常出现；没有任何条件允许为消除此项在R0解锁。

## 7. 拆桨飞行前核查

每项记录实测值、时间及判定人；失败即停止升级。下面命令是后续检查用，不是本轮执行记录。

|检查|可执行的只读方法|应看到什么 / 停止条件|
|FC状态|`timeout 5 rostopic echo -n 1 /mavros/state`|连接且R0未ARM；断连不视为实际已DISARM|
|GPS|`rostopic type /mavros/global_position/raw/fix`；`timeout 5 rostopic echo -n 1 /mavros/global_position/raw/fix`|类型NavSatFix，数值有限、状态有效；还须现场判断精度/延迟，不能只凭status>=0|
|EKF来源|`rosservice call /mavros/param/get "param_id: 'EK3_SRC1_POSXY'"`|本代码要求3，另需核对真实UWB GPS工作状态；不要由本程序写参数|
|EKF报文|`rostopic type /mavlink/from`；`timeout 5 rostopic echo -n 1 --filter 'm.msgid == 193 and m.sysid == 1 and m.compid == 1' /mavlink/from`|有效FCU EKF_STATUS_REPORT；Gate必须得到它，不能只看manager的estimator_status|
|原点|`timeout 5 rostopic echo -n 1 /mavros/global_position/gp_origin`|GeoPointStamped有效；缺失时不伪造原点来通过检查|
|位置/速度|`timeout 8 rostopic hz /mavros/local_position/pose`；对velocity_local同样检查|稳定更新，Pose frame严格`map`；速度frame接受map/base_link，但实际插件轴向仍须核对|
|坐标/姿态|`rostopic echo -n 1 /mavros/local_position/pose`|拆桨手移前/右/上，按下面关系检查；四元数有效、姿态合理|
|单控制出口|`rostopic info /mavros/setpoint_position/local`；另查setpoint_raw/local、setpoint_velocity/cmd_vel、setpoint_velocity/cmd_vel_unstamped、setpoint_raw/attitude|只读V0无FC publisher；启用后只能为`/v0_nav_watchdog`，不能混跑旧控制节点|
|遥控器|看`/mavros/state`和`/mavros/rc/in`，由飞手逐档核对|GUIDED进入/退出、AltHold/Stabilize实际档位及应急能力签核；不由软件自动抢回|
|电池/空间|`rostopic echo -n 1 /mavros/battery`并实物检查|电量/电压阈值按真实电池规定，围挡、人员隔离、旋翼净空及备用落点合格|
|采集|核对原话题、进程、磁盘及预演关闭包|R0不为测试自动录包而ARM；用已有离线证据/已批准的拆桨录制检查方法|

ENU中yaw=0沿+x，正yaw由+x转向+y。任务前/右/上转换：`dx=f*cos(yaw0)+r*sin(yaw0)`，`dy=f*sin(yaw0)-r*cos(yaw0)`，`dz=u`。因此yaw0=0时右移应为-y；yaw0=+90°时前移为+y、右移为+x。**不要再做一遍NED轴交换**。不能假设室内UWB坐标的+x一定指向实验室某面墙；用实测yaw与位移共同判断。

GUIDED须通过实机已验证的RC/GCS人工途径进入，不假定某个RC档位天然可用。AltHold控制高度，水平位置仍由飞手控制；Stabilize需要飞手控制姿态方向及油门。两者均不能不经训练就当作定位故障后的万能兜底，切换时的油门/高度响应必须由飞手预先核验。[O4]

### 7.1 固件和速度参数：只读，不在此自动修复

`rosservice call /mavros/vehicle_info_get '{sysid: 0, compid: 0, get_all: false}'`读取身份。当前代码只接受Copter4.5.7及其已审计参数单位，不支持拿新版本`WP_SPD`替代旧名字。

逐项`rosservice call /mavros/param/get "param_id: '参数名'"`记录：

|参数|当前代码接受范围|
|---|---|
|WPNAV_SPEED|20 ≤ 值 ≤ `motion.speed_xy_mps*100` cm/s；默认YAML要求恰为20|
|WPNAV_SPEED_UP / DN|10至`max_speed_z_mps*100` cm/s；默认10–20|
|WPNAV_ACCEL / ACCEL_Z|50–100 cm/s²|
|GUID_OPTIONS|0|
|AHRS_EKF_TYPE / EK3_ENABLE / EK3_SRC1_POSXY|3 / 1 / 3|

再由飞手核查实际GPS配置、RC/电池/EKF/GCS失效策略及参数含义，**不复制SITL的`FS_GCS_ENABLE=0`**。任一缺失、固件不符、单位不明或限制不符，登记交飞控负责人，不在本轮改参数。

0.20 m/s是期望巡航，不是瞬时硬上限；SITL有约0.26 m/s峰值。软件默认对水平>0.30 m/s或垂直绝对值>0.20 m/s持续0.8秒中止；它不是实体限速器或防撞保护，不因此缩小净空。

## 8. 任务控制租约与完整操作顺序

### 8.1 必须理解的源码时序

`ControlLease.step()`（`v0.py:413`）不是在DISARM时直接授予任务所有权：

1. Gate收到**manager=IDLE、飞控DISARM、无Gate故障**时，置内部`observed_idle=True`，清除旧租约。这是地面准备，不是允许起飞。
2. `/v0_nav/start`只在已人工ARM、GUIDED、定位/参数/原点合格且悬停稳定时接受，冻结P0/yaw0并转`HOLD_INITIAL`。
3. Gate随后看到活动任务、既有地面观察记录、ARM/GUIDED及所有检查通过，才令`owner=mission_id`并转发manager意图。

`observed_idle`本身不发布；`healthy=true, owner=null, revoked=false`也不是可验证的完整“租约已授予”回执。应在地面保持IDLE/DISARM和新鲜state，观察多次正常Gate输出；任务启动后才核对owner。此可观测性限制已列入审阅清单，不虚构`/v0_nav/enable`或“grant lease”服务。

### 8.2 按阶段执行，不在空中切换只读/控制

|次序|操作人及动作|必须看到 / 约束|
|---|---|---|
|1|操作员选定并审核YAML，启动采集/唯一MAVROS，再启动只读V0|第5–7节全部核查；R0禁止ARM|
|2|另行批准R1：保持只读，飞手人工悬停/进入GUIDED；如要预览任务，可显式start|只读没有FC目标；任务预览仍要求健康与稳定悬停，不是绕过检查|
|3|结束R1，飞手人工降落、实际DISARM；操作员确认bag关闭，再Ctrl-C本轮只读导航|只停止导航终端，不动采集/MAVROS；不得悬停中重启Gate|
|4|另行批准R2：在**地面DISARM**状态用下方命令启动控制版两节点|manager IDLE、control_enabled=true；Gate健康、未撤销，owner仍为null；保持若干正常周期|
|5|飞手确认场地、原有保持能力与接管开关，人工ARM/起飞到安全悬停|无软件ARM/起飞；操作员监控采集起包及定位|
|6|飞手人工选择已验证的GUIDED，稳定至少配置hover_time_s|默认2秒内速度、位置/偏航稳定；切GUIDED后尚未start，V0不会发目标，飞手须有已验证保持/接管能力|
|7|操作员经飞手口头同意，调用一次start|success=true，初始target=P0，HOLD_INITIAL约2秒；owner随后等于mission_id|
|8|观察EXEC_WAYPOINT/DWELL/FINISH_HOLD、位姿和实际运动|目标约10Hz；到达需连续1.5秒满足容差，再停留hold_s；不只看publish成功|
|9|FINISH_HOLD后飞手接管、人工降落/DISARM|完成不自动降落；取消任务后也不自动降落|

第4步控制版启动命令（终端N、工作目录`NAV_WS`，**仅后续另行飞行授权后**）：

```bash
roslaunch indoor_nav indoor_nav_v0.launch \
  mission_file:="$MISSION_FILE" evidence_dir:="$SESSION_DIR/v0_nav" \
  enable_control:=true dry_run:=false
```

两把开关在节点构造时读取；`rosparam set`不能动态启用，也不要只重启其中一个节点。YAML的`dry_run_default:true`是schema安全约束，不覆盖launch的显式开关。

第7步及监控命令（检查终端，已加载相同环境）：

```bash
rosservice call /v0_nav/start '{}'
rostopic echo /v0_nav/status
# 分开的观察终端，或依次使用-n 1：
rostopic echo -n 1 /v0_nav/watchdog
rostopic echo -n 1 /v0_nav/origin_pose
rostopic echo -n 1 /v0_nav/target_pose
timeout 5 rostopic hz /mavros/setpoint_position/local
```

`success:true`的message为`origin frozen; initial hold; no mode request`。`success:false`常见于健康、参数快照、GUIDED或悬停门槛不满足；检查status中的`reason/health/gate`，不反复并发start。双start被拒绝**不表示第一项任务已停止**。Gate在IDLE时虽已注册控制publisher，但不发目标；有publisher不等于有运动指令，更不等于FCU执行确认。

## 9. 完成、停止、故障与重新授权

默认生产策略`control.exit_action: manual_takeover`：撤销后续任务并停发，不自动切BRAKE、RTL、LAND或GUIDED。**停止发布不等于飞控立即停止，最后目标可能仍被执行。** 飞手优先按已验证模式接管，不能等ROS服务成功才应急。

|情形|操作与当前源码行为|
|---|---|
|正常完成|进入FINISH_HOLD，仍检查健康并持续最后目标；飞手接管后降落。仅此状态支持`rosservice call /v0_nav/stop '{}'`，进入STOPPED|
|任务执行中要取消|飞手先/同时接管，操作员`rosservice call /v0_nav/abort '{}'`；锁存ABORT_LATCHED并清target。`stop`在未完成时返回false，不是通用急停|
|人工切出GUIDED|manager/Gate撤销；人工切回GUIDED也不自动续飞、不抢回模式；继续由飞手控制并结束架次|
|manager退出/卡死|Gate可检测心跳/status/target超时并撤销；默认不切模式，不保证刹停|
|Gate退出/卡死|唯一出口失效，没有另一软件发布者兜底；依靠飞控侧已核验failsafe及RC，不在空中重启Gate接管|
|MAVROS掉线|旧接收状态不再有效，无法保证还能发任何备用动作；不把connected=false当作实际已落地/上锁|
|定位坏/跳点/原点变更|中止且不重捕获P0继续，不盲目用当前位置保持或BRAKE；由飞手执行定位无效时也适用的已验证应急程序|
|只读时执行abort|也会锁存任务状态，但无飞控命令；以后仍需地面reset/新start|

SITL用例显式启用的`brake_if_position_healthy`不是生产默认值；需要另行审批且满足独立健康条件才可能单次请求BRAKE。即使SetMode `mode_sent=true`也不等于模式已切换或实际停稳。[O3]

重新任务：先由飞手**实际落地DISARM**，确认连接及state新鲜，再检查原因。仅无残留Gate/Health锁存问题时可`rosservice call /v0_nav/reset '{}'`；预期`IDLE; wait for new stable telemetry and explicit start`，还要等待新遥测、重新地面准备和新start。该服务只重建manager的Mission/Health，不直接清Gate健康故障；若仍报原点/跳点/时钟/配置异常或任一进程重启，**在地面停止并重启整套两节点**，查清原因，不自动恢复。绝不以空中重启替代人工接管。

## 10. 声学录包与导航数据对应

### 10.1 本地已查到的recorder，不冒充机载最新版

上述onboard副本`auto_bag_recorder.py`：订阅`/mavros/state`，收到`armed=true`且未录制时执行`rosbag record -o <save_dir>/<bag_prefix> <topics...>`；收到`armed=false`则SIGINT关闭，最多等30秒，超时再terminate。launch默认`save_dir=$HOME/bagfiles`、`bag_prefix=flight_data`；脚本无参数时prefix默认反而为`drone_flight`。运行时以实际私有参数和日志为准。

`-o`由rosbag生成含时间的文件名，不固定等于`flight_data.bag`；用实际目录匹配。录制中可见`.bag.active`，正常关闭后是`.bag`。脚本没有按`connected`单独判定，也未持续检查子进程健康，**失联/异常state可能提前结束或漏录**；不是磁盘成功落盘/音频完整的证明。这些实机待核对，不在本文修改采集代码。

先找实际recorder节点/参数：

```bash
rosnode list
rosparam list | grep -E '(save_dir|bag_prefix|topics)$'
# 将下面节点名换成实际名字：
rosparam get /auto_bag_recorder/save_dir
rosparam get /auto_bag_recorder/bag_prefix
rosparam get /auto_bag_recorder/topics
df -h "$BAG_DIR"
find "$BAG_DIR" -maxdepth 1 -type f -name "${BAG_PREFIX}*.bag*" -ls
```

预期ARM后新活动包出现并增长，DISARM后关闭；不要在R0为触发recorder而ARM。不能用某个硬编码`.active`路径不存在就断定录制失败：先看进程、真实命名和目录。也不能只看“保存完成”日志就忽略磁盘/关闭异常。

### 10.2 保留原话题，优先额外录导航小包

保留全部原音频/振动/ESC/IMU/RC/电池话题，包括真实系统的`/motor_mic/audio_raw`、`/sound_sensor/audio_raw`等；消息类型和音频协议须查实机，不能用SITL mock的UInt8MultiArray作真实类型声明。本地历史launch未含IM72，且录的是`/mavros/local_position/velocity`，与V0所用`velocity_local`不同，需人工补核，**不替换旧列表**。

如尚未批准修改机载recorder，操作员可在同一Master额外手动录导航小包（不是启动第二个MAVROS，也不管理原自动录包）。在任务start前启动，单独终端B、工作目录`NAV_WS`：

```bash
rosbag record -O "$SESSION_DIR/${EXPERIMENT_ID}_nav.bag" \
  /v0_nav/status /v0_nav/event /v0_nav/watchdog /v0_nav/heartbeat \
  /v0_nav/origin_pose /v0_nav/target_pose /mavros/setpoint_position/local \
  /mavros/state /mavros/local_position/pose /mavros/local_position/velocity_local \
  /mavros/global_position/raw/fix /mavros/global_position/gp_origin
```

结束后仅Ctrl-C这个手动bag终端，不停止原recorder。若统一录包，待维护者审核真实launch后，将上述话题**追加**到完整旧列表；本地旧launch没有`topics` arg，不能假定`roslaunch ... topics:=...`有效，也不向运行中recorder设置参数后宣称已热更新。

同一实验编号用于bag_prefix、SESSION_DIR和YAML mission_id，保留YAML SHA及对应关系。`/v0_nav/event`含wall/ROS时间、monotonic增量、ID/seq；两个节点分别存JSON日志，manager另存loaded.yaml/provenance。`heartbeat.seq`不是每帧递增的计数器，不用于宣称丢帧率。音频若为无Header数组，bag记录时间只是接收/录制时间，不是每个样本的精确硬件采样时刻；保留原采样协议及已有时间字段，不伪造同步精度。

## 11. 实验结束与导出

1. 飞手完成实际降落和DISARM，操作员核对新鲜state、遥控器/实物及安全断动力状态。失联时不能只凭最后一条state认定安全。
2. 等自动bag正常关闭；查活动文件、原recorder日志/进程和磁盘。不要在`.active`仍写入时拔电或拔存储。
3. 停止手动导航bag并等待关闭，再Ctrl-C导航终端；原采集应继续，确认后才按其既有SOP停止采集/唯一MAVROS，最后由Master所有者决定是否停Master。
4. `rosnode list`、`pgrep -af 'mavros_node|rosbag|relative_mission_manager|v0_nav_health'`与`lsof "$FCU_DEVICE"`核对本次进程及串口释放。无关任务仍在时不killall/pkill；不使用SITL stop脚本清理实机。
5. 对**关闭后的实际bag文件**执行`rosbag info --yaml "$CLOSED_BAG"`和`rosbag check "$CLOSED_BAG"`。核对时长、声学/ESC/振动/导航话题、数量、时间范围和已知序号/丢块诊断；check通过只说明格式/迁移兼容，不证明PCM无丢样。
6. 保留声学bag、导航bag、两节点日志、loaded.yaml/provenance、审阅后的YAML/源代码SHA、飞控参数只读快照、RC模式记录及后续由飞控负责人导出的BIN。飞控导出方法按其既有流程，不给本轮另开串口。
7. 在SESSION_DIR写实验索引：实验编号、飞手/操作员/观察员、时间及ROS Master、代码commit/dirty、mission_id/YAML SHA、bag实际路径、BIN编号、UWB/EKF/速度参数、异常及接管记录。对归档文件`sha256sum`，复制而非移动/覆盖；传输后重算hash。不要向连接真机的Master执行`rosbag play`。

## 12. 常见问题速查

|现象|检查与处理，不绕过门槛|
|---|---|
|找不到包 / import失败|source顺序、`rospack find`、完整源码及catkin构建；不要只拷一个脚本或混用旧devel|
|MAVROS connected=false|Master/网络、已核对的FCU URL、权限、物理线缆、单端口占用；不要再开第二MAVROS尝试|
|缺GPS/EKF或原点|真实UWB GPS/EKF、插件及`/mavlink/from` 193消息、源ID=1/1；只看estimator_status不够，不注入伪造健康/原点|
|FCU_LIMITS_NOT_VERIFIED等|固件只接受4.5.7、服务/参数缓存、实际速度/EKF参数；等待正常参数读取，失败交负责人，不让程序静默写FC|
|POSE_FRAME_INVALID / POSE_RX_TIMEOUT|分别核对map坐标系和真实回调断流/网络/资源；RAW_STAMP_*只是诊断，不用改消息头或放宽超时掩盖|
|GATE_RESTART_DURING_MISSION / revoked|地面IDLE/DISARM准备是否完成、是否空中重启；飞手接管后落地查因，重启两节点再授权|
|OTHER_*_PUBLISHER / DUPLICATE_MANAGER|查ROS图、旧ExternalNav/其它控制节点或重复launch；只处理本次/获授权进程，不能强抢控制|
|start失败 / stop失败|先看reason/health/gate；start失败不绕过健康，stop只支持FINISH_HOLD，活动任务取消用abort并人工接管|
|录包不出现 / .active不关闭|真实参数路径/命名、ARM状态及断连、磁盘/权限、子进程；不要覆盖原文件或自动修复唯一证据|
|shutdown后飞机仍运动|停发不是刹停；按飞手/飞控预验证应急处置，不期待重启软件自动恢复|

## 13. 对照来源

接口依据为当前本地源码，关键位置见 [代码速查](CODE_AND_CONFIG_GUIDE.md)。历史说明按各自日期保留：Fast250接收时间报告、动态SITL报告、停流排查及原分级SOP；相互差异见审阅清单。

官方补充资料仅用于安装/模式边界，不能覆盖本地schema或代替实机签核：

- [O1] ROS项目Noetic结束支持公告：`https://discourse.openrobotics.org/t/ros-noetic-end-of-life-may-31-2025/43160`
- [O2] MAVROS官方1.20.1说明及GeographicLib安装：`https://github.com/mavlink/mavros/blob/1.20.1/mavros/README.md`
- [O3] ArduPilot Guided说明与命令接口：`https://ardupilot.org/copter/docs/ac2_guidedmode.html`；`https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html`
- [O4] ArduPilot人工模式说明：`https://ardupilot.org/copter/docs/altholdmode.html`；`https://ardupilot.org/copter/docs/stabilize-mode.html`

**本文完成后仍需人工审阅，不因有部署命令就获得启动实机或飞行授权。**
