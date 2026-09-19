# Contract B — Current-aware Terminal Feasibility

日期：2026-09-19。状态：**设计合同；未实现 trim solver 或生产修复**。
依据：[terminal replay 诊断](terminal_handoff_replay_diagnosis.md)。
本合同不改变现有成功条件、海流、capture guard 或任务预算。

## 1. 速度和任务语义

ENU 世界 z 向上、正 pitch 为 nose-up；u/w 是 body 表达的绝对对地速度。
运动学为 x_dot=u cos(theta)+w sin(theta)，z_dot=u sin(theta)-w cos(theta)。
相对水速度是 [u,w] 减 body-frame current。BRAKE 始终使用对地速度；
不能将随水漂移解释为停稳，也不能将“经过目标”解释为完成保持任务。

必须区分两个问题：

- 现有任务：在位置、速度、pitch-rate 容差内持续满足原有限 settle 时间。
- 精确 station keeping：恒流下存在固定位置和固定姿态的平衡。

不存在精确 trim，不自动证明有限容差、有限保持任务不可行，也不排除其他
时变控制的有界运动。因此不能给现有任务偷偷增加“必须存在精确 trim”的拒绝条件。
未来若任务明确要求 stationary equilibrium，才采用对应的可行性硬门槛。

## 2. Trim 问题与约束

给定位置、恒定 world current、Physics 参数版本及有效限值，求
eta_dot=0、nu_dot=0，未知量为 theta、tau_u、tau_m；tau_w=0。
由于当前 J 可逆，静止条件要求 u=w=q=0，而非 nu_relative=0。
用实际 Physics 的完整残差检查 Coriolis、阻尼、恢复项与 current 转换，
不得遗漏无执行器的 heave 平衡方程。

theta 必须位于声明的终端姿态域，同时满足输入限制、q 限制及安全边界。
若任务指定水平保持，则 theta=0 是额外约束；不能为求得解而静默取消。
求解容差按位置速度/加速度/角加速度分别定标，数值阈值与搜索域在运行前冻结。
最小二乘返回成功不等于残差满足平衡；所有原始分量需独立报告。

当前 diagonal synthetic 模型提供一个可用于单元测试的解析特例：
在 nu=0、q=0 时，heave 平衡只剩相对 heave 阻尼，因此正阻尼要求 w_relative=0。
纯垂向流 [0,c_z] 下，w_current=-c_z cos(theta)，c_z 非零时需要 cos(theta)=0。
所以在当前 capture ±60° 参考姿态域内不存在这种精确静止 trim。
这是特定模型/姿态域的解析结论，不是已执行数值搜索，也不是所有 UUV 的普遍结论。

## 3. 结果必须三值化

- FEASIBLE：返回 theta、零绝对速度、平衡控制、完整残差、限值裕量与适用域。
- INFEASIBLE：附解析证明或覆盖整个声明搜索域的可信排除证据。
- UNKNOWN：优化不收敛、搜索不完备、参数域不支持或数值精度不足。

局部求解器找不到解只能返回 UNKNOWN。结果必须标注模型、current、姿态域和
任务语义，不能将“固定水平不可行”扩大为“允许任意姿态仍不可行”。
拟议 TERMINAL_HOLD_INFEASIBLE 仅用于明确要求上述平衡且已有不可行证据的任务；
当前 NavigationStatus 尚无此值。有限保持任务继续按原实际执行结果判定，
可附 equilibrium_infeasible 诊断，但不改写其成功/失败状态。

## 4. 可行 equilibrium 的未来接入

可行解的 reference 为目标世界位置、theta_trim、u=w=q=0，可能需要非零推力。
不得把相对流速填成绝对 u_ref 来冒充静止。姿态/控制需通过受限、安全的过渡到达；
静态平衡可行不保证从当前状态可达、局部稳定或控制器能够跟踪。

完整 trim feedforward 应通过显式的控制能力接口注入，并与已有恢复力矩前馈
明确分账，避免同一恢复项补偿两次。反馈与总前馈合成后使用真实有效限值，
anti-windup 使用同一限值。接口、过渡控制与稳定性实验属于后续实现，不在本轮添加。

## 5. 验收和记录

先验证零流、水平流、纯垂向流解析特例与姿态受限/不受限的差异；
记录 equilibrium residual、控制裕量、求解域、算法状态、证据类型和版本。
再独立验证可行 trim 的真实闭环保持、模型失配与 approach 到 trim 的安全可达性。
测试必须包含“求解失败返回 UNKNOWN”与“无精确 trim 不篡改有限保持任务”两项。
不延长 60 秒 capture 预算，不修改 current，不将原 10/16 结果回填。
