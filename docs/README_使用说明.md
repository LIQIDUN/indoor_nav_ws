# 使用说明

本包包含两份 Markdown 文件和一份 YAML 样例。文件均为**开发输入和架构设计**，不是已完成的代码或 SITL 报告。

## 推荐使用方法

1. 下载并解压本包，放到本地容易找到的目录，例如 `~/Downloads/ArduCopter_V0_Codex/`。
2. 在 Codex 中打开 `~/github/indoor_nav_ws`，确保 Codex 能读取这三个文件（如果在 Downloads，向 Codex 给出该绝对路径；必要时复制到工作区外侧可读的文件夹）。
3. 将 `Codex_开发提示词_ArduCopter_V0相对航点.md` 的全文粘贴给 Codex，并告诉它其余两个文件的路径。
4. Codex 应**先检查 Git 工作树并创建新分支**，再修改项目代码。请避免先把设计文件复制到仓库导致工作树出现不必要改动。
5. F0/F1/F2/F3 的运行结果和真机前置条件以 Codex 实际测试报告为准。不要仅凭本设计文件直接实飞。

**架构约束**：唯一 MAVROS；禁用 pymavlink；ROS1 采集与导航解耦；真机不自动 ARM/起飞/降落。
