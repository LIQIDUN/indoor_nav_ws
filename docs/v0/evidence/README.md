# V0 验证索引与本地原始证据

本目录同时存在 Git 跟踪的小型证据和仅在开发机保存的原件。2026-11-23 整理不重跑实验、不操作真实飞机，
不删除、移动或覆盖原始文件。完整分类见 [整理报告](../GIT_DOCS_AUDIT_20261123.md)；
[逐文件 TSV](GIT_DOCS_INVENTORY_20261123.tsv) 记录当时全部 692 个未跟踪文件的路径、字节数、
原 Git 状态、分类、处理理由和 SHA256。路径均相对工作区根目录，`纳入Git` 与 `本地保留并忽略` 明确区分。

## Git 中提供的验证材料

- [研究版验证记录](../V0_RESEARCH_VALIDATION_20261103.md)：179 项全套、9 项核心、101 项最终 V0 子集；
  [软件测试日志索引](v0-research-20261103/validation-log-index.json) 同时记录早期失败与重试，完整日志只在本地。
- [SITL 总索引](v0-research-20261103/sitl-index.json)：11 次动态尝试，10 PASS / 1 FAIL；另有 3 次只读探针，不能合并计数。
  各动态目录的 `summary.json`、`inputs.json`、`mission.yaml` 和 `cleanup.json` 提供实际输入、源码哈希、结果与退出状态。
  `inputs.json` 的 `git_dirty` 和旧 HEAD 是运行时快照；最终实现随后进入本地 `d684028`。
- [失败摘要](20261103T133514-manager_exit/summary.json)：`POSE_RX_TIMEOUT` 发生在 Manager 退出注入之前。
  pose 最大真实接收间隔 0.973590 s，velocity 0.973584 s；根因尚未确认。随后重试 PASS 不撤销该 FAIL。
- [最终 DISARM 只读摘要](20261103T141057-readonly-streams/summary.json)：level/research 均 PASS；
  [首次失败](20261103T135113-readonly-streams/summary.json) 与 [日志采集误报](20261103T140730-readonly-streams/summary.json) 也保留。
  这是隔离 SITL 中的生产 Launch 集成，不是实际机载 USB/UWB 或实飞验收。
- [停流复盘](../SITL_STALL_REVIEW_20261028.md) 对应的 `stall-review-20261028/` 保留诊断、时序、完整性、身份及历史清单；
  [精简资源摘要](stall-review-20261028/resource_summary.json) 绑定完整本地资源分析的哈希，省略重复进程采样。
  历史清单中引用的运行目录、二进制和工具不全部随 Git 提供。
- `v0-research-20261103/` 中三个 `*-smoke.json`、`strict-baseline.sha256` 和类型审计记录保留配置及接口边界。
  `archive-SHA256SUMS`、`deploy-SHA256SUMS`、`final-files.sha256` 保留历史身份。

## 原件保存位置与获取方式

保存主机：本地 Ubuntu 开发台式机。工作区根目录：`/home/liqidun/github/indoor_nav_ws`。
所有原件仍在原路径，未重新打包或迁移。当前仅确认这台机器上的本地保存状态；本次未创建异地备份或公开下载地址。
其他开发者需要向仓库维护者 LIQIDUN 按以下相对路径和 TSV 中的 SHA256 索取副本；不能通过 Git clone 获取所有原件。

| 本地路径（相对工作区） | 内容与保存状态 |
|---|---|
| `docs/v0/evidence/20261103T*/` | 11 次动态 SITL、3 次只读检查；顶层小型结果进 Git，telemetry/receptions/events JSONL、节点副本及 stdout 原样本地保存 |
| `docs/v0/evidence/20261028T180543-yaw/`、`20261028T180821-yaw/` | 停流对照 PASS/FAIL；摘要与输入进 Git，原始 bag/遥测本地保存 |
| `docs/v0/evidence/resume-suite-*/` | results.json 进 Git；每例启动/测试/停止 stdout 本地保存 |
| `docs/v0/evidence/stall-review-20261028/` | 完整资源采样、Gazebo 统计、console、source_probe、host audit、resource_analysis.json 和 observe.py/analyze.py 本地保存；克隆后运行复盘命令须先取得这些材料 |
| `docs/v0/evidence/v0-research-20261103/` | 两份 tgz、working-tree.patch、完整测试/构建日志、只读探针和过程记录本地保存 |
| `docs/evidence/sitl_flight/` 及其他历史 `docs/v0/evidence/` 案例 | 既有 bag/BIN/tlog、历史运行及重复归档继续保存在原处；已跟踪内容不取消跟踪 |

两份可重建包的历史身份如下。重新生成新包可能因文档版本或压缩元数据而不同，不覆盖旧包：

| 文件（位于 `docs/v0/evidence/v0-research-20261103/`） | 字节数 | SHA256 |
|---|---:|---|
| `v0-research-deploy.tgz` | 47,580 | `3b153481d82fff52c65372936537549dad0fb715a8ed9e79c8e0030c63a3d829` |
| `validation-sources.tgz` | 18,559 | `8d4462cdf47897eb5d37452649c3bdbf2eecf4ca05e37c8ebd6192cda762ac98` |

在原开发机上核对包（只读，不执行部署）：

```bash
cd /home/liqidun/github/indoor_nav_ws
sha256sum -c docs/v0/evidence/v0-research-20261103/archive-SHA256SUMS
sha256sum docs/v0/evidence/20261103T133514-manager_exit/receptions.jsonl
```

第二条输出应与 TSV 对应行的 `sha256_before` 一致。索引中的大小和哈希是整理时读取的原件值，不能证明远端已有副本。

## 校验清单的时间边界

`final-files.sha256` 包含当时运行源码、测试、文档及本地探针的哈希；`deploy-SHA256SUMS` 对应历史部署包成员。
2026-11-23 整理前这两类清单对应的工作树文件身份已核对；随后仅修改必要文档的版本/证据说明。
因此当前文档可能不再匹配历史清单，这是正常版本演进，不更新历史哈希来掩盖差异。
新部署必须按 [部署指南](../V0_RESEARCH_DEPLOYMENT.md) 从审核后的源码生成新包和新清单。

历史 manifest 中的本地 runtime 路径、旧源码哈希及“当时未提交”等注记保持原样，按相应日期解释。
`stop_exit_code: null` 表示该摘要未保存套件停止码，不能解释为 0；套件 `results.json` 与 cleanup 应分别阅读。
任务撤权、DISARM 清理、停止发送目标和 SITL BRAKE 均不等于真实飞机物理刹停或飞行放行。
