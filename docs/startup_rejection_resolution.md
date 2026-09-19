# 启动拒绝原因与修正

日期：2026-09-18。状态：完成原因分解、策略修正和完整 16 场景回归。
本修改发生在 `navigation-v0.8-frozen-candidate-20260918` 之后；冻结版本和原证据保持不变。

## 原三项拒绝并非同一种问题

| 场景 | 原拒绝 | 可观测事实 | 判断 |
| --- | --- | --- | --- |
| initial_speed | drift budget | 初始 u=0.8 m/s，首段 pitch=0；0.15 s 后移动 0.112 m，仍有 u=0.696 m/s | 航向已经对齐，却被统一“停稳再转向”流程误拒绝 |
| zigzag | attitude/rate guard | 首段 pitch=-66.80°，初态静止 | 超出旧 ±60°命令范围，但仍接近原 70°实际保护；属于过保守接口范围 |
| U-trap | attitude/rate guard | 首段 pitch=-81.87°，初态静止 | 真正的近垂直能力边界，不应通过简单删除阈值解决 |

旧错误文本把“路径航向超限”和“实际姿态/角速度超限”合并，容易误以为 UUV 已发生
姿态发散。实际上 U-trap 和 zigzag 都在 t=0、未执行任何控制步时被拒绝。

## 正式修正

### 已对齐状态的有界释放

仅在启动策略第一次检查时，如果同时满足：

- 首段航向误差 ≤0.03 rad；
- |q|≤0.02 rad/s，|w|≤0.03 m/s；
- -0.03≤u≤1.0 m/s；

则返回 `RELEASED_ALIGNED`，路径时钟立即正常推进。它适用于静止或已沿首段正向运动的
车辆，不执行无意义的停车。超过 1.0 m/s、反向运动、横向速度较大或未对齐时不会旁路，
仍进入原 BRAKE→ALIGN→RELEASED 流程。该修改没有改初态、瞬移、重置积分或放宽安全检查。

因此 initial_speed 不再用 0.10 m 转向漂移预算衡量本来无需转向的制动过程。

### 分离且有限的姿态范围

- 启动参考航向范围：±70°；
- 启动期间实际姿态保护：±75°；
- 实际 |q| 保护仍为 0.35 rad/s；
- 姿态参考速率仍为 0.15 rad/s，且严格低于实际 rate guard；
- 配置验证要求 reference limit < actual guard < 90°。

失败信息现在明确区分：

- `startup heading outside configured envelope`：路径首段超出已支持参考范围；
- `startup actual attitude/rate guard`：实际状态越过保护。

这允许 -66.80° 的 zigzag 首段接受真实启动，同时继续拒绝 -81.87° 的 U-trap。
拒绝 U-trap 是明确范围决定，不是遗漏：当前 timed 轨迹在探索性 ±85°启动实验中曾使
任务实际 pitch 达到约 90.5°；zigzag 后续甚至达到约 110.5°并碰撞。启动阶段保护不会
自动约束释放后的轨迹跟踪，因此在建立全任务姿态合同前，不把该探索范围正式采用。

探索目录 `artifacts/navigation/startup_rejection_resolution_20260918/` 保留上述反例，
但不是正式修正的验收证据。

## 修正后完整回归

正式证据：`artifacts/navigation/startup_rejection_resolution_v2_20260918/`。
16 场景各运行两次并通过诊断一致性检查；manifest 零不匹配；全仓 **663 passed
（17.41 s）**。

| 场景 | 修正前 | 修正后 | 解释 |
| --- | --- | --- | --- |
| initial_speed | STARTUP_FAILED, 0.15 s | SUCCESS, 23.50 s | 已对齐初态直接释放 |
| zigzag | STARTUP_FAILED, 0 step | COLLISION, 41.50 s | 启动完成 12.15 s；真实后续能力问题被揭示 |
| U-trap | STARTUP_FAILED, 0 step | STARTUP_FAILED, 0 step | 保留，但原因改为精确的 heading envelope |

总结果由 9 SUCCESS / 3 STARTUP_FAILED 变为 **10 SUCCESS / 1 STARTUP_FAILED**；
另有 3 COLLISION、1 OUT_OF_BOUNDS、1 CAPTURE_FAILED。成功率提升不是本文主要验收条件；
关键是去除两项错误/过保守拒绝，并保留一个有依据的能力边界。

zigzag 的首次碰撞发生在 41.50 s、startup 后的 `after_corner` 阶段；当时 cross-track
约 -0.608 m，reference lead 约 -0.015 m，pitch moment 为有效上限约 73.6%。
它不是启动阶段碰撞，也不是长时间参考领先的简单例子，后续应纳入 progress-based
guidance 比较，而不是继续放宽启动角度。

其他原有失败类别保持：double-baffle 和垂向流碰撞、edge-forced 越界、surge 2 N
案例捕获失败。正常水平、上下斜线、detour、narrow passage 和三组水平流回归成功。

## 测试覆盖与剩余边界

专项测试覆盖：制动先行、姿态参考限速、连续稳定、预算/漂移拒绝、实际运动段碰撞、
释放时积分连续、参考时钟冻结、水平/上下斜线、已对齐 0/0.5/0.8/1.0 m/s 释放、
超过 release surge 不旁路，以及 66.80°/70°/81.87° 航向边界。

本修改不支持任意近垂直或翻转运动，不是 6-DOF 可行性证明，也没有解决 zigzag 碰撞。
U-trap 若未来必须执行，应先定义全任务姿态范围、角度表示、控制/恢复力矩有效性和
后续每段的姿态保护，再作为独立能力升级；不能只把 `maximum_pitch` 调到更大。
