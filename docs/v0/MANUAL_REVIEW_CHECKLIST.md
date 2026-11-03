# V0实机部署前人工审阅清单

日期：2026-10-28。分支：`feat/arducopter-v0-relative-waypoints`。核对HEAD：`dcf3919d6b14d5531e4044c892bc4572e8875ffd`。

**本清单全部实机项待确认，不预先勾选。** 本轮只读源码/历史材料并新增三份文档，没有构建、启动ROS/SITL/Gazebo、连接飞控、试飞、修改参数、stage或commit/push。本次完成文档不等于R0已通过，更不等于可带桨执行航点。

配套：[部署手册](REAL_AIRCRAFT_DEPLOYMENT.md)（简称D）、[代码配置速查](CODE_AND_CONFIG_GUIDE.md)（简称C）、[原分级SOP](REAL_AIRCRAFT_SOP.md)。阅读顺序建议D→C→本表。

## 1. 必须由人确认的部署项

|状态|编号 / 待确认事项|对应章节 / 需要留下的证据|
|---|---|---|
|[ ]|R01 机载Ubuntu、架构、Python、Noetic、MAVROS版本及维护安排|D2–3；实际版本清单，Noetic已EOL的维护责任人|
|[ ]|R02 NAV_WS/ACQ_WS实际目录、批准源码SHA和交付方式|D2、3.3；不覆盖既有工作区，未提交文档不能假设已随clone交付|
|[ ]|R03 underlay/overlay顺序与同名包遮蔽|D3.3、5.1；实际rospack路径及构建结果，源码/devel布局是否保留|
|[ ]|R04 唯一ROS Master、ROS_IP/HOSTNAME和时钟|D5；实际地址与双向可达性，use_sim_time不是true，无SITL路由遗留|
|[ ]|R05 Pixhawk型号、Copter固件、system/component|D7.1、C4.4；实机是否为4.5.7及1/1。不同则不能绕过当前校验|
|[ ]|R06 FCU真实设备路径、连接线/波特率/权限|D4–5；udev/by-id、lsof/PID，GPS接入和伴随电脑连接不混淆|
|[ ]|R07 真实最新版采集launch及所有include|D5.3；保存绝对路径、SHA、启动MAVROS次数；不使用旧上传副本覆盖|
|[ ]|R08 XVF3800实际协议与采集参数|D4、10；实际采样率/位宽/通道/分块/增益/话题，保留已验证16kHz设置而非重新调参|
|[ ]|R09 IM72D128六通道实际驱动与入口|D4、10；当前本地所查launch无该节点，机载48kHz等配置/类型/完整话题列表待核对，不能由mock替代|
|[ ]|R10 振动/ESC/IMU/RC等原话题完整性|D7、10；逐个type/hz/录包覆盖，不能用新导航列表替掉原列表|
|[ ]|R11 UWB GPS与EKF稳定性|D4、7；拆桨定位精度/延迟/噪声、GPS质量、EKF flags/ratio、原点是否稳定；不调查底层协议|
|[ ]|R12 拆桨ENU/yaw/上下方向|D7；手动前/右/上位移及机头变化与pose对照，禁止起飞试方向|
|[ ]|R13 MAVROS必要话题/服务存在且类型匹配|D7、C5；尤其velocity_local、gp_origin、/mavlink/from的193报文；只看estimator_status不够|
|[ ]|R14 一个MAVROS与唯一Gate控制出口|D5、7；ROS图+主机PID+设备占用证据，不仅同名节点列表|
|[ ]|R15 Gate启动/地面准备/任务授权顺序|D8、C2.1；先IDLE/DISARM观察，后人工ARM/悬停/GUIDED/start，活动后owner=mission_id；不空中重启|
|[ ]|R16 固件速度/加速度/EKF参数及单位|D7.1、C4.2；只读参数快照，实际人工已设置合格；0.20m/s不当瞬时硬上限|
|[ ]|R17 遥控器GUIDED进入方式及AltHold/Stabilize接管|D7–9；档位/人工进入途径、飞手训练、定位失效时可用应急策略，不假设GUIDED必在某档|
|[ ]|R18 默认manual_takeover与飞控failsafe|D9、原SOP；manager/Gate/MAVROS死亡时飞控实际策略、RC可用性；不照搬SITL BRAKE/FS_GCS_ENABLE=0|
|[ ]|R19 电池、场地、人员分工和净空|D7、原SOP；飞手/操作员/观察员/负责人、隔离区及应急落点，不用软件围栏替代物理防护|
|[ ]|R20 第一份实际任务YAML|D6、8；独立单个0.2–0.3m前向、z=0等方案经审核，P0/yaw0与起始EKF高度含义正确，默认5点不直接实飞|
|[ ]|R21 自动recorder真实版本/参数/目录|D10；实际节点名、磁盘挂载/剩余空间、topics、prefix、实际文件名；R0禁止为了起包ARM|
|[ ]|R22 录包异常与关闭流程|D10–11；ARM起包、DISARM关闭、子进程/磁盘故障检测及失联时行为，由真实版本核实|
|[ ]|R23 导航记录追加方法|D10.2；保留原话题，人工批准追加或额外手动导航bag，检查原点/事件与音频时间关联|
|[ ]|R24 日志和实验对应/导出|D11；YAML/源码SHA、Git dirty、声学bag/导航bag/BIN编号及人员签名，关闭包检查与传输后hash|
|[ ]|R25 真机联合测试资格|原SOP R0→R4；真实硬件联合测试仍未完成；模拟负载PASS不能勾选实物R4|

## 2. 需要人工裁决的源码/历史材料差异

以下只登记，不在本轮悄悄修改源码或旧报告。并非每项都要求改代码，可选择保留限制并按正确顺序部署。

|编号|已核实差异 / 风险|源码或文档位置|建议决定，等待人工确认|
|---|---|---|---|
|Q01|“DISARM就授予租约”不准确：只是observed_idle准备；活动且ARM/GUIDED/无故障才授予owner。内部准备位未公开|`v0.py:413`；`v0_nav_health.py:253`、`:263`|确认D8时序，决定是否接受现有可观测性；不虚构grant服务、不在本轮加接口|
|Q02|旧SOP已要求地面启动，但随后“人工悬停后显式开启两把开关”的措辞可能被读成半空启用；参数实际只在构造时读取|`REAL_AIRCRAFT_SOP.md:62`、`:64`；manager`:34`、Gate`:30`|以D8明确顺序审阅：只读转控制必须先落地，再地面重启两节点。旧文不覆盖，后续人工批准再修措辞|
|Q03|YAML dry_run_default不是动态开关；两节点需要同一SHA和control_enabled；只重启单节点会不匹配|`v0.py:97`；`v0_nav_health.py:188`|批准静态启动方式；不使用rosparam热切换，见C3–4|
|Q04|reset只重建manager，不清Gate Health所有锁存，如原点变更/跳点；Gate还需再次观察地面IDLE/DISARM|manager`:259`；`v0.py:310`、`:340`、`:414`|D9采用地面查因，必要时整套重启；不把reset当一键恢复|
|Q05|只支持4.5.7和旧WPNAV单位；原始EKF固定筛选源1/1；任意新固件或MAVROS namespace不能直接套用|`v0.py:200`；manager`:153`；Gate`:133`、`:150`|实机核对；若不符，另行立项兼容/验证，不修改版本门槛骗过检查|
|Q06|manager还消费estimator_status，但Gate只靠有效原始EKF_STATUS_REPORT；IDLE时Gate healthy不代表所有飞行健康检查已通过|manager`:80`；Gate`:133`、`:196`|确保真实MAVROS raw报文、GPS和原点可用，拆桨核查；D7不把healthy=true等同放行|
|Q07|同名all_sensors.launch内容不同：onboard副本含MAVROS，ros_sensor副本仅振动；所查onboard入口不含IM72|外部两份路径见D5.3|提供真实机载最新版及其SHA，决定唯一连接入口；不覆盖已验证采集系统|
|Q08|onboard旧采集launch录`/mavros/local_position/velocity`，V0实际用`velocity_local`；旧列表无导航/IM72话题|外部onboard `all_sensors.launch:61`|人工审阅后追加实际存在的话题，或另录导航bag，不删除原列表|
|Q09|旧launch没有topics arg；recorder仅初始化读取私有参数，不支持通过roslaunch未知arg或运行中改param自动更新|外部`all_sensors.launch:53`；`auto_bag_recorder.py:15`|核对真实实现后选最小追加方式；D10未提供覆盖命令|
|Q10|本地recorder看armed而非connected，启动后未持续poll子进程；“保存完成”不证明磁盘及PCM完整|外部`auto_bag_recorder.py:42`、`:52`、`:76`|确认真实版本是否已处理，制定操作员监控/关闭检查；本轮不改采集代码|
|Q11|当前包仍含nlink_parser等旧构建依赖，provenance依赖源码/devel相对布局；只复制脚本或纯install部署未等价验证|`CMakeLists.txt:3`；manager`:46`、`:60`|采用完整源码工作区/devel部署；若要精简/安装空间部署，另行授权验证|
|Q12|源码默认5点与SITL四点不是同一份配置；start/stop/reset用途不能混同|`mission_v0.yaml:45`；`tools/v0_sitl_test.py:35`；manager`:233`、`:251`|批准独立首飞单点配置；双start拒绝不等于原任务停止，stop不是急停|
|Q13|历史Fast250报告的“暂停”、早期F2/GAP文档是当时快照；专项Fake GPS模拟已取消|Fast250离线报告开头、`FAKE_GPS_INTERFACE_GAP.md`、开发计划修订|不要当作新的F2阻塞要求；真实UWB质量留R0，禁止新开协议研究|
|Q14|后续无GUI停顿仍有FAIL；旧报告没有证明是GUI/高CPU导致。按本轮决定作为仿真基础设施已知限制存档|`SITL_STALL_REVIEW_20261028.md:3`及原summary|不重开研究、不重写FAIL/因果、不将该限制自动外推为实机可靠或不可靠；由负责人裁决其对后续分级放行的影响|
|Q15|默认manual_takeover仅停发；SITL BRAKE成功和单次SetMode返回不是生产通用刹停保证|`mission_v0.yaml:12`；Gate`:222`；安全说明|在飞控/RC独立应急未签核前禁止升级带桨航点，不因文档齐全视作关闭风险|

## 3. 当前Git状态快照

本轮三文档完成时应保持：

- 分支`feat/arducopter-v0-relative-waypoints`；HEAD仍为`dcf3919d6b14d5531e4044c892bc4572e8875ffd`。
- 已跟踪文件无修改；暂存区为空；工作区**dirty（有未跟踪文件）**。没有commit、push、merge、rebase或自动stage。
- 本轮之前已有的未跟踪资料/证据全保留，只新增以下三份Markdown；不能把它们统称“本轮修改的飞行代码”。

```text
?? docs/ArduCopter_V0相对航点_完整项目设计与验收文档.md
?? docs/Codex_开发提示词_ArduCopter_V0相对航点.md
?? docs/README_使用说明.md
?? docs/mission_v0_copter.yaml
?? docs/v0/CODE_AND_CONFIG_GUIDE.md                  本轮新增
?? docs/v0/MANUAL_REVIEW_CHECKLIST.md                本轮新增
?? docs/v0/REAL_AIRCRAFT_DEPLOYMENT.md               本轮新增
?? docs/v0/SITL_STALL_REVIEW_20261028.md              既有未提交
?? docs/v0/evidence/20261028T180543-yaw/              既有未提交
?? docs/v0/evidence/20261028T180821-yaw/              既有未提交
?? docs/v0/evidence/resume-suite-20261028T180535-1501338/
?? docs/v0/evidence/resume-suite-20261028T180812-1504415/
?? docs/v0/evidence/stall-review-20261028/
```

此为2026-10-28快照，后续归档前重新运行`git status --short`、`git diff --stat`、`git diff --cached --stat`确认，不凭本文替代实时状态。`.gitignore`隐藏的bag/BIN/tlog/runtime存在于本地，不会显示在该清单，不代表已纳入Git。

本轮完成检查只做文档/接口静态核对与既有文件哈希保护核验；没有重跑SITL或将历史单测当作本轮执行。人工审阅仍可提出修改，未确认内容保持未勾选。

## 4. 下一次人工批准后建议归档范围

|分类|实际范围|建议|
|---|---|---|
|生产代码|manager、Gate、v0.py及原ExternalNav源码|本轮无差异，已有HEAD已记录；不要为归档文档重复修改/提交代码|
|配置|`mission_v0.yaml`、V0 launch、SITL参数；未来经审核的实机单点YAML|本轮未改；只归档实际批准的配置及其SHA，不把SITL参数当实机配置|
|文档|本轮三份手册/清单|人工逐份审阅后优先归档；旧报告保持历史原样|
|既有待归档报告|`SITL_STALL_REVIEW_20261028.md`|单独审阅后可与对应证据归档；不把未关闭历史FAIL改写为PASS|
|测试工具|既有tools及`stall-review-20261028/observe.py`、`analyze.py`|原生产工具本轮没改；后两者是已执行排查的证据脚本，不列为实机入口|
|轻量证据|两次yaw目录的summary/inputs/mission/cleanup/events/receptions/telemetry等；两suite记录；stall-review的分析、资源/统计、manifest、source_probe、审计文件|按实际体积/内容选择，检查源码身份与FAIL保留；不要无审查git add整目录。这些目录是前轮证据，不是本轮实验|
|大体积二进制|flight.bag、.bag.active、BIN、mav.tlog/.raw、EEPROM、`.sitl_runtime/`、build/devel|**不建议纳入Git**；在独立存储保留原路径映射、大小、SHA256与索引。活动包不能当正常关闭包归档|
|原始用户输入|上面docs/下四份原始说明/YAML|**不默认提交**，不改写；独立保留，是否入库由用户决定，不能被模板覆盖|
|外部声学源码|真实机载采集工作区和本地历史副本|不从本仓库覆盖或顺手提交；如需快照/差异补丁，另行明确范围|

归档前按清单逐项选择文件、检查`git diff --cached`及是否包含设备隐私/大文件；**本轮不执行任何stage/commit，也不提供一键git add .方案**。二进制保留不等于删除，未提交文档也应继续保存在工作树。

## 5. 签核记录

|角色|姓名 / 日期|结论、未关闭问题与允许的下一步|
|---|---|---|
|代码/部署审阅人|待填写|三份文档与实际机载版本差异|
|采集负责人|待填写|最新版launch、IM72/XVF/recorder与数据归档|
|飞控与定位负责人|待填写|固件/EKF/UWB方向/参数/独立failsafe|
|飞手与场地负责人|待填写|RC模式、隔离区、应急程序及分级放行|
|用户最终确认|待填写|仅归档文档 / 允许后续拆桨调查 / 其它明确范围|

**当前无实机条目已完成的声明。三份文档交用户人工审阅后停止，不自行部署、启动实验或提交。**
