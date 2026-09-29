# Terminal Approach v1 固定 P1 路径对照

2026-09-29。配置在首轮闭环前冻结，首轮通过后仅扩大场景集合，没有根据结果调整参数。
独立分支 `codex/terminal-approach`；P2b 留在 `c9d809e`，本分支不包含其适配器。

## 验证范围与结果

先执行三场景×两臂×两次（12 次），再执行 12 场景×两臂×两次（48 次）。
路径直接读取经过 manifest 核验的 P0/P1 冻结 P1 几何，不重新规划、不改变起点。
每次使用同一 PID/前馈、Physics、LOS、capture、startup、安全检查和 120 s 时限。
关闭 approach 的全部 P1 history 与旧存档逐步一致；所有新旧实验臂的双重复一致。
壁钟开销不参与确定性比较。全仓 **847 passed**（本分支基线 829 + 新增 18 项）。

| 最终状态 | P1 | P1 + approach |
| --- | ---: | ---: |
| SUCCESS | 7 | 10 |
| CAPTURE_FAILED | 2 | 0 |
| TIMEOUT | 1 | 0 |
| COLLISION | 2 | 2 |
| OUT_OF_BOUNDS | 0 | 0 |

这是独立的零流、0.2 m/s P1 12 场景实验，**不是**原 LOS 0.5 m/s 16 场景的更新；
原 LOS 10/16 保持不变。

| 场景 | P1 → approach | 时长 s（P1 / 新） |
| --- | --- | ---: |
| horizontal | SUCCESS → SUCCESS | 42.65 / 48.00 |
| ascending | CAPTURE_FAILED → SUCCESS | 45.60 / 59.60 |
| descending | CAPTURE_FAILED → SUCCESS | 45.60 / 59.60 |
| initial_pitch | SUCCESS → SUCCESS | 48.70 / 54.05 |
| initial_speed | SUCCESS → SUCCESS | 42.60 / 48.30 |
| saturation | SUCCESS → SUCCESS | 43.80 / 48.50 |
| narrow_passage | SUCCESS → SUCCESS | 42.65 / 48.00 |
| detour | SUCCESS → SUCCESS | 63.95 / 68.75 |
| double_baffle | TIMEOUT → SUCCESS | 120.00 / 98.30 |
| edge_forced_detour | COLLISION → COLLISION | 35.65 / 35.65 |
| u_trap | SUCCESS → SUCCESS | 73.95 / 78.60 |
| zigzag | COLLISION → COLLISION | 27.50 / 27.50 |

原成功场景都成功，但通常更慢，不声称性能全面改善。提前低速接近是显式、独立的
末段策略，不是改 PID 或全路径 corner slowdown。

## double_baffle：预测、接管与实际交接

- 67 s 开始上游评估，剩余弧长 3.994 m；所有候选未通过，继续原跟随。
- 69 s 仍为 UNKNOWN；有限时域未合格不代表物理不可行。
- **71 s**，剩余弧长 3.121 m，选择速度 0.10 m/s、制动触发距离 0.20 m 的候选。
  首次 approach 状态：位置 `(7.5126, -7.5198)` m，pitch `0.6110` rad，
  `(u,w,q)=(0.2213,0.0448,0.2282)`；不是直到几何 readiness 才制动。
- **98.25 s** 实际三步资格通过并交给原 capture：位置 `(8.8456,-4.8589)` m，
  距目标约 0.209 m，pitch `0.9055` rad，`(u,w,q)=(0.01198,-0.01784,0.01159)`。
  预测/实际六维交接状态差为零；执行中每步均检查预测偏差。
- **98.30 s** 成功；终点误差 0.2101 m、对地速度 0.02137 m/s、q 0.00399 rad/s。
  原成功保持时间 0.5 s 未改；保持可在 approach 内开始累计。
  capture 已经处于 `ORIGINAL_SETTLE`，并未执行额外局部位置修正。
  因此本次证明合格交接/稳定到达，不是证明 capture 能修复任意剩余误差。
- 饱和持续时间从 **3.25 s 降到 0.20 s**（任一轴达到限值的总时长）。
- 全程最小障碍净余量仍为 **7.20 mm**，早期路径执行未改变，不能称鲁棒安全。

原 detour 最小净余量 0.8643 m 未变；新策略保持零饱和，耗时增加 4.80 s。
两个碰撞案例在评估区前失败，预测步数均为零；净距侵入仍为 2.34 / 1.82 mm。

## 计算量、诊断与局限

double_baffle 共三次评估、8,991 个预测物理步；首轮累计预测壁钟约 7.38 s，
完整对照 repeat 1 约 8.97 s。上下斜线各 9,120 步；简单成功场景约 473–527 步。
这些是当前机器/并发负载下的观测，不满足 0.05 s 实时周期的证据。
真实任务仿真时间不因离线预测计算暂停而增加；不把这种运行方式宣称为实机可用。

每次候选记录完整起始状态、PID 积分记忆、剩余弧长、候选动作、预测终态和失败原因。
实际每步仍执行原连续碰撞/边界检查。预测副本隔离、资格计数重置、搜索预算、
非零流拒绝、真实状态偏离预测及 Navigator progress 所有权均有测试。
零流同模型预测准确不代表模型失配/传感器噪声鲁棒性；5 mm 预测裕量不是误差证明。

未改变 capture guard、未扩大任务预算、未增加终点容差；但本实现只是 Contract A 的
**LOS、零流、有限候选子集**。timed 接入、current-conditioned equilibrium、预测模型误差
与实时计算尚未实现。仍保持 opt-in，P1 默认行为不变。

`final_path_progress` 和旧 `maximum_pitch_reference_step` 仅对应原 LOS command，
approach 开始后原 cursor 冻结，不能用这些旧字段冒充全阶段进度/参考连续性指标。
静态图展示实际轨迹、实际姿态/rate 和施加力矩，不用于单凭图片判断碰撞安全。

## 证据

- `artifacts/terminal_approach_v1/`：先导配置、源码哈希、12 次历史、逐候选诊断、manifest。
- `artifacts/terminal_approach_full_v1/`：完整 48 次历史和独立 summary/manifest。
- `artifacts/terminal_approach_full_report_v1/`：12 张固定路径配对图、统计和 manifest。
- `artifacts/terminal_approach_final_tests.xml`：847 项测试的 JUnit 记录。
- [冻结索引](terminal_approach_v1_dataset.json)：manifest 校验和；不替代原始本地 artifacts。

首个提交点可包含本次有限实现和证据索引。随后可独立进入 ROS 2 Jazzy adapter 的
接口/回放设计；机器人类型、命令能力仍待确认，不要求先修完两个碰撞场景。
