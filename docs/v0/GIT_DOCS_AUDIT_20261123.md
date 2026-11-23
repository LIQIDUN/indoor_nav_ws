# Git 文档与实验产物整理（2026-11-23）

范围：`/home/liqidun/github/indoor_nav_ws`，分支 `feat/arducopter-v0-relative-waypoints`。整理起点为本地 `d684028`；没有拉取或推送远程，没有运行仿真、接触真机或修改导航源码。

## 基线与分类方法

只读检查：暂存区为空、已跟踪文件无修改；docs 共 2002 个文件、1,675,327,789 B（du 约 1.6 GiB）。其中 1310 个已跟踪、692 个未跟踪；未跟踪中 261 个原本在普通状态可见，431 个已由现有规则忽略。未发现需要保留的无关暂存条目。主机检查没有活动 ROS/SITL 构建或仿真进程；已有 GitHub Desktop 未操作。

V0-Research 实现及重点文档此前已进入 `d684028`，不是本次新开发或尚未提交源码。本次保留原有文档，修正开发阶段“未提交”与当前本地版本状态的歧义；远程分支包含哪些提交未查询。

判断依据是内容、大小、重复性和复现价值。所有 692 个起始未跟踪文件逐个列在 [分类与校验索引](evidence/GIT_DOCS_INVENTORY_20261123.tsv)，含完整仓库相对路径、字节数、原 Git 状态、分类、处理方式、理由和 SHA256。该索引记录整理前的原件，不承诺外部备份；新增整理文件由本次 Git 提交记录。

## 长期文档逐项处理

以下文件在整理开始前均已跟踪。历史报告按日期理解，不将其旧计划或快照视作当前部署验收；没有为整理而删除历史文档。

| 文件 | 原大小 | 原 Git 状态 | 分类 | 处理方式 | 理由 |
|---|---:|---|---|---|---|
| `docs/ACCEPTANCE_REPORT.md` | 6,650 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/ArduCopter_V0相对航点_完整项目设计与验收文档.md` | 38,168 B | 已跟踪、未修改 | 设计/接口/安全 | 保留已有跟踪，不重复提交 | 说明配置契约、控制职责和限制 |
| `docs/Codex_开发提示词_ArduCopter_V0相对航点.md` | 22,979 B | 已跟踪、未修改 | 历史需求/决策 | 保留已有跟踪，不重复提交 | 保留任务范围、取消项与当时状态 |
| `docs/EXTERNALNAV_EKF_FUSION_REPORT.md` | 5,322 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/FREEZE_INVESTIGATION.md` | 11,379 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/MAVLINK_EXTERNALNAV_FIX_REPORT.md` | 12,802 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/README_使用说明.md` | 1,047 B | 已跟踪、未修改 | 使用/部署 | 保留已有跟踪，不重复提交 | 复用部署与人工核对步骤 |
| `docs/SITL_DEBUG_AND_FLIGHT.md` | 5,365 B | 已跟踪、未修改 | 使用/测试方法 | 保留已有跟踪，不重复提交 | 保留无 GPS 仿真检查与人工操作流程 |
| `docs/SITL_FLIGHT_VALIDATION_REPORT.md` | 7,303 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/SITL_STALL_INVESTIGATION.md` | 3,665 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/SITL_STARTUP_GUIDE.md` | 8,333 B | 已跟踪、未修改 | 使用/部署 | 保留已有跟踪，不重复提交 | 复用部署与人工核对步骤 |
| `docs/UWB_EKF_ACCEPTANCE_SUMMARY.md` | 5,105 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/UWB_VELOCITY_CAPABILITY.md` | 2,299 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/UWB_VELOCITY_EVALUATION.md` | 3,521 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/VELOCITY_FRAME_FIX_REPORT.md` | 6,780 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/mission_v0_copter.yaml` | 1,793 B | 已跟踪、未修改 | 设计/接口/安全 | 保留已有跟踪，不重复提交 | 说明配置契约、控制职责和限制 |
| `docs/v0/00_REPO_AUDIT.md` | 5,712 B | 已跟踪、未修改 | 历史需求/决策 | 保留已有跟踪，不重复提交 | 保留任务范围、取消项与当时状态 |
| `docs/v0/CODE_AND_CONFIG_GUIDE.md` | 20,880 B | 已跟踪、未修改 | 使用/部署 | 保留已有跟踪，不重复提交 | 复用部署与人工核对步骤 |
| `docs/v0/DEVELOPMENT_PLAN_REVISION_20261026.md` | 8,154 B | 已跟踪、未修改 | 历史需求/决策 | 保留已有跟踪，不重复提交 | 保留任务范围、取消项与当时状态 |
| `docs/v0/FAKE_GPS_INTERFACE_GAP.md` | 3,243 B | 已跟踪、未修改 | 历史需求/决策 | 保留已有跟踪，不重复提交 | 保留任务范围、取消项与当时状态 |
| `docs/v0/FAST250_DYNAMIC_SITL_REPORT_20261028.md` | 19,328 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/v0/FAST250_RECEIPT_TIME_REVIEW_20261028.md` | 13,319 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/v0/FOUR_WAYPOINT_ROOT_CAUSE.md` | 3,804 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/v0/MANUAL_REVIEW_CHECKLIST.md` | 13,291 B | 已跟踪、未修改 | 使用/部署 | 保留已有跟踪，不重复提交 | 复用部署与人工核对步骤 |
| `docs/v0/PAUSE_REPORT_20261027.md` | 23,810 B | 已跟踪、未修改 | 历史需求/决策 | 保留已有跟踪，不重复提交 | 保留任务范围、取消项与当时状态 |
| `docs/v0/README.md` | 20,091 B | 已跟踪、未修改 | 使用/部署 | 必要版本/证据说明修订后提交 | 复用部署与人工核对步骤 |
| `docs/v0/REAL_AIRCRAFT_DEPLOYMENT.md` | 33,086 B | 已跟踪、未修改 | 使用/部署 | 保留已有跟踪，不重复提交 | 复用部署与人工核对步骤 |
| `docs/v0/REAL_AIRCRAFT_SOP.md` | 10,096 B | 已跟踪、未修改 | 使用/部署 | 保留已有跟踪，不重复提交 | 复用部署与人工核对步骤 |
| `docs/v0/SAFETY_AND_LIMITATIONS.md` | 4,862 B | 已跟踪、未修改 | 设计/接口/安全 | 保留已有跟踪，不重复提交 | 说明配置契约、控制职责和限制 |
| `docs/v0/SITL_REPORT.md` | 10,603 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |
| `docs/v0/SITL_STALL_REVIEW_20261028.md` | 8,175 B | 已跟踪、未修改 | 验证/故障结论 | 必要版本/证据说明修订后提交 | 保留证据边界、失败和未解决项 |
| `docs/v0/V0_RESEARCH_DEPLOYMENT.md` | 11,987 B | 已跟踪、未修改 | 使用/部署 | 必要版本/证据说明修订后提交 | 复用部署与人工核对步骤 |
| `docs/v0/V0_RESEARCH_DEPLOY_FILES.txt` | 675 B | 已跟踪、未修改 | 使用/部署 | 保留已有跟踪，不重复提交 | 复用部署与人工核对步骤 |
| `docs/v0/V0_RESEARCH_UPGRADE.md` | 13,520 B | 已跟踪、未修改 | 设计/接口/安全 | 必要版本/证据说明修订后提交 | 说明配置契约、控制职责和限制 |
| `docs/v0/V0_RESEARCH_VALIDATION_20261103.md` | 11,103 B | 已跟踪、未修改 | 验证结论 | 必要版本/证据说明修订后提交 | 保留测试数量、失败、重试和未测边界 |
| `docs/v0/WEB_REVIEW_STATUS_20261026.md` | 23,158 B | 已跟踪、未修改 | 历史需求/决策 | 保留已有跟踪，不重复提交 | 保留任务范围、取消项与当时状态 |
| `docs/v0/WORKSPACE_REPORT_20261103.md` | 9,605 B | 已跟踪、未修改 | 验证/故障结论 | 保留已有跟踪，不重复提交 | 保留证据边界、失败和未解决项 |

## 本次纳入 Git 的历史小型证据

共 75 个原有小型文件、406,773 B。单文件最大 44,646 B；运行摘要最多约 33 KiB，无 bag/BIN/tlog/遥测 JSONL/完整 stdout。每例保存 summary、inputs、mission 和 cleanup 是为了同时保留案例事实、源码哈希、真实测试输入和清理边界；这些内容不能只由最终生产 YAML 重建。

| 文件 | 大小 | 原 Git 状态 | 分类 | 处理方式 | 理由 |
|---|---:|---|---|---|---|
| `docs/v0/evidence/20261028T180543-yaw/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261028T180543-yaw/inputs.json` | 1,644 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261028T180543-yaw/mission.yaml` | 1,533 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261028T180543-yaw/summary.json` | 31,338 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261028T180821-yaw/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261028T180821-yaw/inputs.json` | 1,644 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261028T180821-yaw/mission.yaml` | 1,533 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261028T180821-yaw/summary.json` | 1,598 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T132555-single/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T132555-single/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T132555-single/mission.yaml` | 1,595 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T132555-single/summary.json` | 32,987 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T132812-multi/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T132812-multi/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T132812-multi/mission.yaml` | 1,756 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T132812-multi/summary.json` | 33,292 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T133046-abort/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T133046-abort/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133046-abort/mission.yaml` | 1,756 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133046-abort/summary.json` | 9,498 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T133301-mode_exit/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T133301-mode_exit/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133301-mode_exit/mission.yaml` | 1,760 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133301-mode_exit/summary.json` | 9,465 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T133514-manager_exit/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T133514-manager_exit/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133514-manager_exit/mission.yaml` | 1,763 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133514-manager_exit/summary.json` | 2,147 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T133725-pose_loss/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T133725-pose_loss/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133725-pose_loss/mission.yaml` | 1,760 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133725-pose_loss/summary.json` | 9,493 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T133940-gate_kill/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T133940-gate_kill/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133940-gate_kill/mission.yaml` | 1,760 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T133940-gate_kill/summary.json` | 9,335 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T134159-single/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T134159-single/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134159-single/mission.yaml` | 1,595 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134159-single/summary.json` | 32,992 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T134415-multi/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T134415-multi/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134415-multi/mission.yaml` | 1,756 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134415-multi/summary.json` | 33,278 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T134649-manager_exit/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T134649-manager_exit/inputs.json` | 1,850 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134649-manager_exit/mission.yaml` | 1,763 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134649-manager_exit/summary.json` | 9,442 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T134906-single/cleanup.json` | 150 B | 未跟踪 | 清理结果 | 提交 | 独立记录DISARM和进程退出；不是飞行通过证明 |
| `docs/v0/evidence/20261103T134906-single/inputs.json` | 1,841 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134906-single/mission.yaml` | 1,536 B | 未跟踪 | 验证输入 | 提交 | 绑定实际源码/配置与案例；小型不可替代输入 |
| `docs/v0/evidence/20261103T134906-single/summary.json` | 32,998 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T135113-readonly-streams/summary.json` | 95 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T140730-readonly-streams/summary.json` | 308 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/20261103T141057-readonly-streams/summary.json` | 9,650 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/resume-suite-20261028T180535-1501338/results.json` | 314 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/resume-suite-20261028T180812-1504415/results.json` | 314 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/resume-suite-20261103T132546-237930/results.json` | 2,240 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/resume-suite-20261103T134150-245361/results.json` | 960 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/resume-suite-20261103T134857-248492/results.json` | 318 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/stall-review-20261028/dataflash_diagnostic.json` | 5,114 B | 未跟踪 | 诊断摘要 | 提交 | 支撑故障结论的小型结构化分析或类型审计 |
| `docs/v0/evidence/stall-review-20261028/gui-false-identity.json` | 259 B | 未跟踪 | 诊断摘要 | 提交 | 支撑故障结论的小型结构化分析或类型审计 |
| `docs/v0/evidence/stall-review-20261028/gui-true-identity.json` | 258 B | 未跟踪 | 诊断摘要 | 提交 | 支撑故障结论的小型结构化分析或类型审计 |
| `docs/v0/evidence/stall-review-20261028/integrity.json` | 1,083 B | 未跟踪 | 诊断摘要 | 提交 | 支撑故障结论的小型结构化分析或类型审计 |
| `docs/v0/evidence/stall-review-20261028/manifest.json` | 44,646 B | 未跟踪 | 诊断摘要 | 提交 | 支撑故障结论的小型结构化分析或类型审计 |
| `docs/v0/evidence/stall-review-20261028/timing_analysis.json` | 30,094 B | 未跟踪 | 诊断摘要 | 提交 | 支撑故障结论的小型结构化分析或类型审计 |
| `docs/v0/evidence/v0-research-20261103/archive-SHA256SUMS` | 254 B | 未跟踪 | 历史校验索引 | 提交 | 历史包/文件身份；不表示对应原件随Git提供 |
| `docs/v0/evidence/v0-research-20261103/deploy-SHA256SUMS` | 1,863 B | 未跟踪 | 历史校验索引 | 提交 | 历史包/文件身份；不表示对应原件随Git提供 |
| `docs/v0/evidence/v0-research-20261103/final-files.sha256` | 2,445 B | 未跟踪 | 历史校验索引 | 提交 | 历史包/文件身份；不表示对应原件随Git提供 |
| `docs/v0/evidence/v0-research-20261103/mavros-stream-type-audit.txt` | 183 B | 未跟踪 | 诊断摘要 | 提交 | 支撑故障结论的小型结构化分析或类型审计 |
| `docs/v0/evidence/v0-research-20261103/mission_v0-smoke.json` | 425 B | 未跟踪 | 离线配置验证 | 提交 | 固定配置哈希和坐标输出 |
| `docs/v0/evidence/v0-research-20261103/mission_v0_level-smoke.json` | 423 B | 未跟踪 | 离线配置验证 | 提交 | 固定配置哈希和坐标输出 |
| `docs/v0/evidence/v0-research-20261103/mission_v0_research-smoke.json` | 432 B | 未跟踪 | 离线配置验证 | 提交 | 固定配置哈希和坐标输出 |
| `docs/v0/evidence/v0-research-20261103/sitl-index.json` | 9,683 B | 未跟踪 | 结果索引 | 提交 | 保留成功失败和重试；小型结构化结果 |
| `docs/v0/evidence/v0-research-20261103/strict-baseline.sha256` | 104 B | 未跟踪 | 历史校验索引 | 提交 | 历史包/文件身份；不表示对应原件随Git提供 |

另生成 `evidence/stall-review-20261028/resource_summary.json`，从完整资源分析中提取两次观测指标并绑定原件 SHA256；生成 `evidence/v0-research-20261103/validation-log-index.json`，记录各次历史软件测试的数量、成功/失败、日志大小和哈希。两者是既有材料的索引，不是重新运行或新增验证。

## 本地保留与忽略

新增规则只针对明确生成路径/文件名；复用现有 bag、BIN、tlog、log、Python 缓存和 Catkin 构建规则。没有忽略整个 docs/v0 或 evidence，没有通配忽略 Markdown/JSON/YAML。忽略规则不取消历史已跟踪文件。

| 范围（均在 docs 下） | 用途 | 本次处理 |
|---|---|---|
| `v0/evidence/20??????T??????-*/` 的 telemetry/receptions/events JSONL、navigation.txt、rosbag.txt、navigation 子目录 | 原始遥测、事件、节点重复配置/日志 | 定向忽略，顶层 summary/inputs/mission/cleanup 保留进 Git |
| 只读探针的 `mission_v0_*.yaml/` 子目录 | 每配置节点运行副本 | 忽略副本，三次探针摘要全部提交 |
| `v0/evidence/resume-suite-*/*-{instance,stop,test}.txt` | 套件 stdout | 忽略，results.json 提交 |
| `evidence/sitl_flight/*/telemetry.jsonl` | 逐样本飞行遥测 | 阻止新增误提交，已跟踪文件不动 |
| `v0/evidence/stall-review-20261028/` 的 gui 日志/资源采样、完整 resource_analysis、source_probe、final_host_audit、两份 Python 工具 | 一次性排查原件 | 定向忽略，诊断/完整性/时序/清单及精简资源结果提交 |
| `v0/evidence/v0-research-20261103/*.tgz`、working-tree.patch | 部署与测试源码副本 | 忽略；校验清单提交 |
| 同目录 initial-status、preexisting-untracked.sha256、readonly-orphan-cleanup、readonly_probe.py | 开发阶段/探针过程原件 | 定向忽略，结果文档和校验索引保留 |

两份包实际为 `v0-research-deploy.tgz`（47,580 B、18 个成员）及 `validation-sources.tgz`（18,559 B、6 个成员）。体积虽小，仍因源码/部署副本可重建而留在本地。`archive-SHA256SUMS` 已核对其身份；不覆盖、不重新生成历史包。

原始实验文件都留在仓库工作树原路径。访问根目录、获取方式、保存状态及校验命令见 [证据说明](evidence/README.md)。本次未创建异地副本；忽略不等于备份。

## 历史已跟踪大文件：仅列建议

以下 >=1 MiB 的 8 个文件均为历史实验归档，当前工作树及 Git 跟踪保持不变。建议未来先确定外部归档位置、校验与获取流程，再由用户批准是否取消跟踪；本次没有 git rm --cached，也没有改写历史。

| 文件 | 字节数 | 分类/建议 |
|---|---:|---|
| `docs/evidence/sitl_flight/run_06/telemetry.jsonl` | 18,843,007 | 历史遥测；未来可评估取消跟踪，本次保留 |
| `docs/evidence/mavlink_after.jsonl.gz` | 11,505,170 | 历史遥测；未来可评估取消跟踪，本次保留 |
| `docs/evidence/sitl_flight/run_05/telemetry.jsonl` | 7,693,520 | 历史遥测；未来可评估取消跟踪，本次保留 |
| `docs/evidence/sitl_flight/run_04/telemetry.jsonl` | 3,065,690 | 历史遥测；未来可评估取消跟踪，本次保留 |
| `docs/evidence/velocity_frame_continuity.jsonl.gz` | 2,829,976 | 历史遥测；未来可评估取消跟踪，本次保留 |
| `docs/evidence/sitl_gui/audit.jsonl.gz` | 2,213,775 | 历史遥测；未来可评估取消跟踪，本次保留 |
| `docs/evidence/sitl_flight/run_03/telemetry.jsonl` | 1,888,806 | 历史遥测；未来可评估取消跟踪，本次保留 |
| `docs/evidence/sitl_flight/run_02/telemetry.jsonl` | 1,860,232 | 历史遥测；未来可评估取消跟踪，本次保留 |

## 实验事实与版本核对

- 已核对历史最终日志：全套 179 PASS、核心 9 PASS、最终 V0 子集 101 PASS；较早子集 100 PASS 是另一轮。首次依赖导入失败与 Launch 断言失败均保留在日志索引，不把重试替换为全程成功。
- 动态 SITL 11 次：10 PASS / 1 FAIL。失败 `20261103T133514-manager_exit` 在注入前发生 `POSE_RX_TIMEOUT`；原 receptions 重新读取确认 pose 最大间隔 0.973590 s、velocity 0.973584 s，根因尚未确认。所有案例 summary 与 sitl-index 结果逐项一致。
- 只读探针另计：两次 FAIL 后最终 level/research PASS；这是隔离 SITL 的 DISARM 只读集成，未验证真实 USB、机载部署或实飞。
- 10 月 28 日 GUI/无 GUI 的 PASS 与 FAIL、尚未确认的 SITL/Gazebo 根因均保持。新归档不增加任何通过结论。
- 所有待提交 JSON/YAML 可解析；案例 mission.yaml 的实际 SHA256 与 inputs.json 一致。最终源码与三份生产配置的历史哈希在整理前全部匹配；源码不修改。
- 选中文档及小型证据未发现凭据模式；保留必要工作区路径、回环端口和仿真默认坐标用于追溯。没有新增真实机载地址/密码或访问真实设备。
- README、升级说明、验证记录、部署指南与停流复盘只补版本/证据边界；SAFETY_AND_LIMITATIONS 和 REAL_AIRCRAFT_SOP 已与科研策略一致，原样保留。

## 提交与保护校验

本次提交范围仅为 .gitignore、上述 5 份修订文档、本报告、证据说明、逐文件清单、小型历史证据和两个派生索引。无 Python/C++ 源码、二进制或原始运行日志，无文件删除。使用明确路径白名单暂存并审查差异。

提交前后以整理前 SHA256 清单核对全部 2002 个原有 docs 文件：允许变化仅为上述 5 份技术文档；全部实验原件和其余文档应字节一致。最终 commit SHA、实际行数及工作树检查结果由任务最终汇报给出，避免报告自引用提交哈希。没有 push。
