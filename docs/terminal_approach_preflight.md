# Terminal Approach 前置诊断

2026-09-22。分支 `codex/terminal-approach` 从 main 建立，不包含 P2b opt-in 改动。
P2b 保留在 c9d809e；本次仅复核冻结历史，不实现或修改生产策略。

## double_baffle 的实际阶段

来源：`artifacts/planner_paired_v1/runs/double_baffle_detour_P1_{1,2}.json`。
已核验原 manifest，并独立重建两次历史的全部 LOS readiness 谓词；结果一致。

- 终止 TIMEOUT，STARTUP 189 步，PATH_FOLLOWING 2,211 步。
- TERMINAL_CAPTURE **0 步**；terminal_ready **0 次**。
- 因此不是“进入 capture 后绕圈”，也不是 capture guard 实际拒绝。

| 观测时刻 | 时间 s | 到目标 m | 剩余弧长 m | 对地速度 m/s | q rad/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 首次进入最后线段 | 82.80 | 0.4955 | 0.6458 | 0.2257 | -0.5519 |
| 末段剩余弧长首次 ≤0.25 m | 84.35 | 0.3012 | 0.2481 | 0.2155 | -0.5927 |
| 最近目标的施令前状态 | 85.25 | 0.2678 | 0.0761 | 0.2135 | -0.6364 |
| 最后一个施令前状态 | 119.95 | 0.5680 | 0 | 0.3745 | -0.6047 |

readiness 同时要求末段、剩余弧长 ≤0.25 m、实际到目标距离 ≤0.25 m。
后两个时刻的前两项通过，但实际距离始终不满足。LOS 继续以 0.2 m/s surge target
追逐有限终点，实际进度耗尽并不触发交接。

最近点的 |q| 已超过现有 capture 0.35 rad/s guard；如果强制交接，会面临另一项
资格问题，但它不是原历史未交接的直接原因。不得混淆几何 readiness 与 admissible handoff。

## 对实现顺序的约束

1. 不放宽 0.25 m 距离门限、0.35 rad/s capture guard、成功条件或任务预算。
2. 不等待 terminal_ready 才制动；它在该历史从未出现。
3. 不能假设“首次进入最后线段”就仍有合格入口：82.80 s 的 pitch 约 1.5298 rad，
   q 约 -0.552 rad/s，已经超出现有 capture 实际姿态/rate guard。
4. 接近阶段的触发评估应在末段上游进行，记录剩余弧长、末段相对误差、姿态、
   u/w/q 和积分记忆；由完整状态预测判断停止空间，而非只使用距离或速度查表。
5. 沿用 Contract A：隔离的控制器/物理预测、每条预测运动段安全检查、有限预测预算、
   连续入口资格计数；无法证明入口资格时返回 UNKNOWN/明确失败，不强行交给 capture。
6. 固定 P1 路径比较旧策略与 approach；不混入 P2b、重新规划、PID 或终端成功语义变化。

Contract A 中入口阈值、资格保持步数、预测误差余量仍未冻结；本诊断没有把一次
观测距离当作生产制动参数，也没有声称 TerminalApproachPolicy 已实现。

## 复现

```powershell
.\.venv\Scripts\python.exe -m validation.terminal_approach_preflight --output artifacts/terminal_preflight_new
```

本次证据：`artifacts/terminal_approach_preflight_v1/results.json` 与 manifest。
JSON 含逐项谓词、关键时刻完整状态/积分诊断、源 SHA-256、双重复一致性。
没有重跑物理闭环，也没有改动原历史。
