# Planner v0.6c — 首轮有限 P0/P1 对照

2026-09-20。零流、固定 0.2 m/s、相同 LOS/Control-v0.5.1/Physics/startup/capture、
原 120 s 任务预算，12 场景 × 两臂 × 两次 = **48 次执行**，逐步重复一致。
原 0.5 m/s LOS **10/16** 不变。本轮禁用 shortcut，不与历史统计直接比较。

## 主要结论

| 指标（每臂 12 个条件） | P0 | P1 |
| --- | ---: | ---: |
| 规划拒绝 | 0 | 0 |
| SUCCESS | 6 | 7 |
| COLLISION | 2 | 2 |
| OUT_OF_BOUNDS | 0 | 0 |
| TIMEOUT | 2 | 1 |
| CAPTURE_FAILED | 2 | 2 |

只有 **detour** 的选中路径变化，且两次均由 TIMEOUT 变为 SUCCESS。
其余 11 场景路径相同、执行结果相同。**本轮没有降低碰撞数**，不能声称普遍避碰改善。

| detour 指标 | P0 | P1 |
| --- | ---: | ---: |
| 路径长度 m | 10.071 | 10.485 |
| 转角数 | 5 | 2 |
| 采样参考障碍净距 m | 0.200 | 0.700 |
| 实际连续运动段障碍净距 m | 0.3666 | 0.8643 |
| 最小实际边界净距 m | 0.0443 | 0.7000 |
| 扩展节点（含全部候选） | 145 | 527 |
| 本机规划时间 ms（执行预备轮观测） | 163.04 | 495.04 |

长度增加约 4.1%，计算开销增加。改进是完整路径形状、启动目标及末段入口共同作用的
结果，不是孤立转角因果证明；不能把成功仅归因于某个权重或已恢复的角点间距。
时间是本机墙钟观测，不是统计性能 benchmark；重复闭环共享各臂已冻结的规划结果。
几何预审与执行预备轮重新规划，忽略墙钟后所有路径、代价及扩展节点一致。

## 实现合同

- `planner/candidates.py` 保留 geometric A*；固定 buffer 为 0/.15/.30/.50/.75 m。
  buffer 只约束候选生成，不改变执行安全阈值。候选拒绝不等于任务物理不可行。
- 长度 + 转角 + 采样净距缺口 + 角点间距缺口 + UNKNOWN 惩罚，选择**总代价最小**。
  间距只用于完成后的候选排序，不声称总代价搜索最优性；不增加不必要的历史搜索状态。
- P0/P1 共用去重和**同向**共线简化；反向点积非正时保留折返，重新连续验证。
- 4 m 优选间距是经验软偏好，不是从 v2 推导的认证恢复距离。
- 逐角点分别保存前视弧长首次触及角点的施令前状态、积分诊断、首次偏离切向命令、切段状态。
  detour 的 P0 五个/P1 两个 preturn 均早于切段。无法匹配孤立实验入口，全部保持 UNKNOWN。
  不因数值间距满足就认定相邻转弯已经恢复。
- 未修改 PID、LOS、Physics、终端生产策略、海流、执行器/安全/终止条件。

## 剩余失败与范围

- ascending/descending：两臂 CAPTURE_FAILED；共享几何 A* 折线路径而非之前的直线 fixture。
- double_baffle_detour：两臂 TIMEOUT。
- edge_forced_detour、zigzag_maze_detour：两臂 COLLISION。
- 成功不等于鲁棒：例如 u_trap 仍只有约 0.071 m 观测障碍净余量。
- 失败样本的误差、时间和 effort 基于截短历史；低累计代价不能解释为更优。
- 本候选家族对 11/12 场景没有改变最终路径。尚无 heading-aware 搜索或动态可行性保证。

## 证据和复现

配置：`config/planner_paired_v1.json`（几何预审之前固定，无结果后调参）。

```powershell
.\.venv\Scripts\python.exe -m validation.planner_paired --stage geometry --output artifacts/planner_paired_new
# 先审核 geometry.json，再执行；不可覆盖现有冻结目录。
.\.venv\Scripts\python.exe -m validation.planner_paired --stage execute --output artifacts/planner_paired_new
.\.venv\Scripts\python.exe -m validation.planner_paired_report --input artifacts/planner_paired_new --output artifacts/planner_paired_report_new
```

- `artifacts/planner_paired_v1`：源文件/配置 SHA、几何代价、48 份独立 history、summary、manifest。
- `artifacts/planner_paired_report_v1`：逐场景规划开销和净距表、事件统计、12 场景闭环对照图。
- `artifacts/planner_paired_tests_final.xml`：全仓 **809 passed**（新增 20 项）；覆盖反向共线、最小代价、
  参数拒绝、UNKNOWN、连续验证与 preturn 早于切段。
- [冻结索引](planner_paired_v1_dataset.json) 仅提供定位和校验，不能替代未入 Git 的 history。

下一步可独立进入 ROS 2 adapter 接口设计，不以修完所有 terminal/current/复杂场景为前提。
本轮不直接实现 adapter，也不自动开始新的规划调参。
