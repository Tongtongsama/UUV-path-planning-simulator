# 可选恢复力矩前馈与一致限幅 / Proposed Control v0.5.1 contract

Status: implemented as optional Control v0.5.1, 2026-09-15; default coefficient
remains zero. See
[formal implementation and acceptance](control_v0.5.1_validation.md). The text
below preserves the original design rationale; its former "not implemented"
statements describe the design-stage status, not the current delivery status.

## 1. Scope and dependencies

新增可选 reduced pitch-restoring compensation；不改 surge/depth 外环、PID gains、
终点策略、轨迹采样或 Physics。控制器只接收数值系数，不读取 WorldModel 或导入
Physics。Simulation/配置层负责提供估计系数并记录其来源。

建议参数：`restoring_pitch_coefficient: float = 0.0`，有限、非布尔、非负。
0 表示关闭；旧 YAML 缺省字段必须按 0 加载，不可自动从 Physics 偷读系数。
默认基线保持原样。synthetic 实验显式设置 6 N·m；真实车辆需辨识或给定来源。

## 2. Effective actuator limits

在本步积分之前确定每轴有效对称上限：

```text
L_u = min(controller_tau_x_limit, navigation_tau_x_limit, actuator_tau_x_limit)
L_q = min(controller_tau_m_limit, navigation_tau_m_limit, actuator_tau_m_limit)
```

未建模执行器时省略其上限。所有上限有限、正值；v0.5.1 不支持非对称、速率、
耦合或推力分配约束，不能把本设计误当作 actuator allocation。

建议公开不可变 `ControlLimits3DOF`，并通过独立的
`LimitAwareController3DOF.step_with_limits(state, reference, dt, limits)` 能力接口
传入本步约束。保留现有 `Controller3DOF.step/reset`；step 使用自身配置上限。
不要强迫旧后端新增不理解的方法。

Navigation 对声明支持该能力的控制器，在调用前传递其最终限幅约束；对旧控制器
保留原行为，但明确标记没有 applied-limit-aware anti-windup 保证。Navigation
最终 clip 可作为防御性检查；能力模式下它应是数值不改变输出的操作。
若外部执行器临时进一步缩小可用范围，必须提前传入，或另行设计 applied-control
feedback；当前提案不声称解决事后未知限幅。

## 3. One combined request, one limit decision

前馈根据实际姿态计算，以抵消当前静态恢复项：

```text
ff = k_restore * sin(actual_pitch)
pitch_PD = Kp * wrap(pitch_target - actual_pitch) + Kd * (q_ref - q)
requested_tau_m = pitch_PD + ff + Ki * I_pitch
applied_tau_m = clip(requested_tau_m, -L_q, +L_q)
```

不先限幅 PD/PI 再加 ff。例：PD=−10、ff=+6、L_q=8，正确输出为 −4，
而不是先把 PD 限成 −8 再得到 −2。控制器自身不输出单独前馈供下游再相加。

surge 没有新增前馈，但同样使用 L_u 做积分判断和最终 clip。

## 4. Conditional integration

对每轴计算候选积分：`I_candidate=clip(I_old+error*dt, -I_limit, I_limit)`。
候选请求必须使用全部非积分项；pitch 使用 `pitch_PD+ff`。

- 若候选请求在有效上限内：接受积分。
- 若候选请求超过正上限，只允许负 error 对积分的更新。
- 若候选请求低于负上限，只允许正 error 对积分的更新。
- 否则冻结积分，使用原 I 重算请求，再按同一有效上限限幅。

保留既有积分容量边界和 reset 语义。Ki=0 时积分不影响输出；保持旧基线更新语义，
不把这一提案扩展为另一种积分器。该方法是 conditional integration，不是
back-calculation；不承诺受限输出立即解除饱和。

当前系数模型最多抵消已知恢复项，不补偿惯性、Coriolis、阻尼、海流或模型误差。
当 ff 本身超过可用力矩，必须保留饱和和姿态误差证据，不能隐藏为“已补偿”。

## 5. Diagnostics and compatibility

建议提供只读 `last_diagnostics`，不改变 ControlInput 的六维物理语义：

- 有效 pitch target 和 wrapped error；
- PD、ff、积分前后值、合成限幅前请求；
- 实际输出、有效上下限、饱和/积分冻结标志。

Navigator/Recorder 在启用此功能时另行保存诊断；原 commanded control 仍是公开
Controller API 的输出，不可悄悄改名为未限幅信号。step_with_limits 不重复调用 step，
否则会使积分器每步更新两次。

关闭 ff、有效上限等于旧上限时，命令和积分记忆应与旧 baseline 逐步一致。
以后改 loader 时，新增可选字段不得破坏旧配置；配置/记录必须包含开关、估计系数、
有效约束和来源。默认关闭。

## 6. Executable design verification

`validation/feedforward_pid_prototype.py` 仅用于验证提案，重复使用基线外环公式，
并在 pitch 条件积分与最终输出前都加入同一个 ff。实验构造时给定已知最终上限。
它不是生产实现，不提供上述尚待实现的动态 limits 协议或完整公共数据类型。

测试覆盖关闭功能的输出/记忆等价性（含角度跨界和饱和）、正负前馈饱和时冻结向外
积分、反向误差解除积分、reset、反馈与前馈先合成再限幅、非法系数和零上限。

正式编码验收还必须补充：旧 YAML 兼容、限幅能力协商、Navigation 更紧上限下的
完整集成、逐步变限幅、raw/commanded/applied 诊断对齐及模型失配对照。
没有这些验证，不能把实验原型等同于已交付的 Control v0.5.1。
