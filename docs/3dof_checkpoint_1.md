# 3DoF Research Checkpoint 1 — 统一交付与演示入口

2026-09-29。交付一个适用范围明确、可复现的仿真研究版本，不是实机控制器或安全认证版本。
算法迭代到此收尾；后续独立开展 ROS 2 Jazzy 接口与回放，不再为本提交点调 PID。

## 版本组成

- 交付分支：`codex/terminal-approach`；算法冻结提交：`f6b6d38`。
- 纯 Python ENU x-z、SNAME body 的非线性欠驱动 3-DoF Physics；Control v0.5.1
  的原 PID、可选实际姿态恢复力矩前馈和一致限幅/anti-windup。
- P1 候选路径 + finite-segment LOS + 既有 startup/capture，作为本交付的评估基线。
  不传 `approach_policy` 时行为不变；不修改库默认，也不将 P1 偷换成曲线路径。
- 显式候选：`config/terminal_approach_v1.json`，通过 runner 的 `APPROACH` 实验臂启用。
  `P1` 实验臂关闭它；调用者亦可显式构造
  `navigation.terminal_approach.TerminalApproachPolicy` 并传给 Navigator。
- P2b 留在独立分支 `codex/p2b-path-representation`、提交 `c9d809e`。
  **P2b + approach 未组合验证，不能计入本次 10/12。**
- artifacts 不进入 Git；完整历史、预测诊断、图表、JUnit 和 frozen manifests 通过
  项目外备份保存。Git 源码及 dataset 索引不能替代这些文件。

## 四组证据不能混算

| 实验 | 条件与对象 | 可报告结论 |
| --- | --- | --- |
| 原 matched LOS / timed | 16 场景，原 0.5 m/s，含流场对照 | LOS 10/16，timed 9/16 |
| P0/P1 paired v1 | 12 零流场景，双方 0.2 m/s | P0 6/12，P1 7/12；只有 detour 选中不同路径 |
| P2b 表示实验 | 同一解析 detour 圆弧，不同采样/表示 | primitive 消除本例密集弦段 gate 退化；12 场景只是关闭适配器后的 P1 回归 |
| TerminalApproach v1 | 同一冻结 P1 路径，12 零流场景，原跟随 0.2 m/s | P1 7/12 → opt-in 10/12；两次重复一致 |

原始报告：[LOS/timed](guidance_matched_16_results.md)、[P0/P1](planner_paired_v1_results.md)、
[terminal](terminal_approach_v1_results.md)。P2b 报告在其独立分支的
`docs/p2b_representation_results.md`，可用 `git show c9d809e:docs/p2b_representation_results.md` 阅读，
无需切换当前工作树。

## 演示顺序（直接展示冻结图表，不必重新运行）

所有相对路径均以仓库根目录为基准。

| 顺序 | 场景 | 演示重点 | 已生成图 |
| --- | --- | --- | --- |
| 1 | horizontal | 基本闭环与停车；新策略仍成功但更慢 | [图](../artifacts/terminal_approach_full_report_v1/horizontal.png) |
| 2 | detour | 同一 P1 路径的规划/跟踪；不混用 P2b | [图](../artifacts/terminal_approach_full_report_v1/detour.png) |
| 3 | double_baffle | 上游塑造终端入口，120 s 超时 → 98.30 s 成功 | [图](../artifacts/terminal_approach_full_report_v1/double_baffle_detour.png) |
| 4 | edge_forced_detour | 保留碰撞作为能力边界 | [图](../artifacts/terminal_approach_full_report_v1/edge_forced_detour.png) |
| 5 | zigzag_maze_detour | 保留密集转弯执行局限 | [图](../artifacts/terminal_approach_full_report_v1/zigzag_maze_detour.png) |

图展示实际轨迹、姿态、角速度、施加力矩，不是障碍扫掠安全证明；安全结论来自
历史中的连续运动段检查和指标。[完整统计](../artifacts/terminal_approach_full_report_v1/summary.json)。
每个场景的原参考/有效指令、状态、积分入口、预测候选及交接诊断位于
`artifacts/terminal_approach_full_v1/runs/<scene>_<P1|APPROACH>_<1|2>.json`。

必须保留的解说：double_baffle 的改善主要由 approach 完成，交接时已达到低速、
低角速度和位置资格；capture 以 ORIGINAL_SETTLE 收尾，没有额外位置修正。
成功保持时间按原全任务语义累计，不人为延长 capture 展示。

## 安装与复现（PowerShell，仓库根目录）

需要 Python 3.10+、本分支源码及冻结 P1 原始工件。先恢复备份，再复跑；runner 会校验
`artifacts/planner_paired_v1/` 的 geometry、history 与 manifest，缺失时不是从零自造替代输入。
以下路径名必须尚不存在，防止覆盖冻结证据；重复复跑请更换所有 `recheck` 目录名。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-validation.txt -r requirements-visualization.txt
.\.venv\Scripts\python.exe -m pytest -q --junitxml=artifacts/checkpoint1_recheck_tests.xml
.\.venv\Scripts\python.exe -m validation.terminal_approach_comparison --output artifacts/checkpoint1_recheck_pilot
.\.venv\Scripts\python.exe -m validation.terminal_approach_report --input artifacts/checkpoint1_recheck_pilot --output artifacts/checkpoint1_recheck_pilot_report
```

上述先导同时运行 horizontal、detour、double_baffle 两臂各两次，无需再建一套演示算法。
若先导通过且配置未改变，再复现 12 场景：

```powershell
.\.venv\Scripts\python.exe -m validation.terminal_approach_comparison --full --gate artifacts/checkpoint1_recheck_pilot --output artifacts/checkpoint1_recheck_full
.\.venv\Scripts\python.exe -m validation.terminal_approach_report --input artifacts/checkpoint1_recheck_full --output artifacts/checkpoint1_recheck_full_report
```

当前冻结测试证据：`artifacts/terminal_approach_final_tests.xml`，847 passed。
这是已执行结果，不代表未来任何 Python/依赖版本都会逐位相同；每次输出 freeze.json 记录源码，
重现时同时核对 Python 与依赖环境。依赖声明是范围约束，不是锁定环境。

## 交付边界与下一阶段

- 两个碰撞仍存在；double_baffle 全程最小净余量仅 7.2 mm，不是鲁棒避碰保证。
- 原七个成功保留，但多用约 4.65–5.70 s；完成率改善有时间代价。
- 预测副本与实际相同模型，不能据此证明模型失配/传感器噪声鲁棒性。
- 单次任务累计预测约 0.4–9.5 s（当前观测），没有 0.05 s 在线实时预算保证。
- 当前 approach 仅支持零流 LOS；不声称 timed/current-aware terminal 已完成。
- 下一步 ROS 2 Jazzy：先接口与日志回放。机器人类型、命令能力、定位与频率待实验室确认。
  不把 x-z pitch 直接改名成地面 x-y yaw，不将推力接口假定为 cmd_vel。
- 本次只交付/备份，不推送 GitHub，不合并 P2b，不开始 ROS 代码。
