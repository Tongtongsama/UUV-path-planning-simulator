# Navigation v0.8 候选版本冻结与统计

冻结标识：`navigation-v0.8-frozen-candidate-20260918`。
这是可追溯的候选基线冻结，不是全场景通过的 release，也没有修改全局默认。
没有自动 commit、创建 Git tag 或推送仓库。

## 冻结边界

- Git 基点：`4e6b08cbbf78aad107cb6cf7ad95e0715f998a82`。
- 原验收：`artifacts/navigation/frozen_candidates_16_20260917/`。
- 冻结与图表：`artifacts/releases/navigation_v08_frozen_20260918/`。
- `freeze.json` SHA-256：`437267c160f98751f56cb7992757d5941ff674fdb2560e6f55bf6eac708bef47`。
- freeze.json 保存逐文件源码 SHA-256、包版本、Git 基点和原始 evidence manifest 的哈希。
  manifest.json 校验本次生成的图表、统计和冻结文件。
- 图表生成前已逐项核对原始证据与记录的源码哈希；没有重跑或改写原场景结果。
- 原完整测试证据为 **655 passed**。本轮只增加只读统计/绘图工具与文档，未修改算法。

冻结组合：A* + 原路径后处理 + timed reference + 现有局部减速候选 +
Control v0.5.1 恢复前馈候选 + ControlledStartupPolicy + TerminalCapturePolicy +
Python 3-DOF Physics。详细参数以逐场景 JSON 和源码哈希为准。
后续 LOS 必须作为新方法/新结果目录，不能覆盖或重命名本版证据。

大体积历史与 PNG/PDF 继续放在被 .gitignore 排除的 artifacts 下，不复制整份 source。
本冻结说明和生成脚本可以版本管理；本地 artifacts 不是远程备份，需要长期保留时应
另外备份两个完整证据目录。Git 基点与哈希不能替代实验数据备份。

## 统计

| 结果 | 数量 | 占固定 16 案比例 |
| --- | ---: | ---: |
| SUCCESS | 9 | 56.25% |
| COLLISION | 2 | 12.50% |
| OUT_OF_BOUNDS | 1 | 6.25% |
| STARTUP_FAILED | 3 | 18.75% |
| CAPTURE_FAILED | 1 | 6.25% |

全部 16 案各执行两次，结果一致。零流 narrow passage 在原套件中重复出现，
不能当作 16 个独立随机样本；不报告统计置信区间或泛化成功概率。
失败时长是提前终止时长，不应与成功任务时间混算为平均完成时间。
没有障碍的 clearance 为 null，不是零；启动拒绝不是成功避碰。

`statistics.json` 保留每案终止状态、物理运行时长、末位置误差/速度、最小障碍净余量、
饱和比例、启动/捕获耗时和 control-effort proxy。后者不是真实能耗。
首次碰撞与边界失败的分段解释见 [完整回归报告](frozen_candidates_16_regression.md)。

## 可视化

- `scenario_atlas.png` / `.pdf`：16 个场景，原 A* 路径、参考、实际轨迹、障碍物、
  侵蚀边界、起点/目标及最终状态。叉号是终止位置，不一定代表碰撞。
- `statistics.png` / `.pdf`：状态数量、终点误差（标出 0.25 m 容差）、观测时长。
- 两张 PNG 已目视核对，无漏场景、混淆终止状态或将拒绝案例画成执行成功。

复现（使用新目录，脚本拒绝覆盖现有输出）：

```powershell
.venv\Scripts\python.exe -m validation.frozen_release_report --evidence artifacts/navigation/frozen_candidates_16_20260917 --output artifacts/releases/navigation_v08_report_new
```

## 下一阶段边界

冻结和可视化完成后，下一轮正式开展目标明确的开源 guidance 调研：

1. UUV Simulator：只检查 interpolator、waypoint/trajectory-point、导数与速度时间参数化。
2. Python Vehicle Simulator/MSS：只检查 LOS、有限线段投影、look-ahead、进度/切段和终点交接。
3. 记录固定 commit、许可证、实际阅读的文件/函数以及 ENU x-z 适配差异。
4. 先形成接口与公平比较规范，再实现独立 LOS/progress-based baseline。

保留现有模块化系统、PID 内环、路径/安全条件和 Physics，不接入完整 simulator；
本轮没有审计上游代码，不声称已采用其实现，也未提前开始 LOS 或平滑开发。

冻结后的启动拒绝修正另见 [startup rejection resolution](startup_rejection_resolution.md)；
该修改不回写本冻结版本的哈希、统计或图表。
