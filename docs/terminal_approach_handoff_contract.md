# Contract A — Terminal Approach / Handoff

日期：2026-09-19。状态：**设计合同；未实现、未验收**。
依据：[terminal replay 诊断](terminal_handoff_replay_diagnosis.md)。
原 LOS 10/16、timed 9/16 统计不变；本合同不修改 guard、预算、current 或 PID。

## 1. 所有权和接口

Navigation 的可选 TerminalApproachPolicy 负责提前减速与交接；
TerminalCapturePolicy 只接收符合入口条件的状态，不负责挽救过晚、高速入口。
阶段顺序为 PATH_FOLLOWING → TERMINAL_APPROACH → TERMINAL_CAPTURE；
已满足原任务成功条件时仍走原保持时间判定，不强制额外转向或捕获。

策略显式接收实际 VehicleState、原路径、有限线段进度、目标、current、dt、
有效执行器限值、控制器记忆快照和停止包络版本。输出阶段决定、
BodyGuidance3DOF 有效目标、入口资格及诊断原因，不直接推进 Physics。
有效目标使用现有 guidance 能力接口绕过位置/depth 外环，保留内环 PID、
实际姿态恢复力矩前馈、最终限幅和一致 anti-windup。阶段切换不得重置积分。

## 2. 提前触发，而不是等待 terminal_ready

进入末段候选区域后，按实际状态和剩余有限路径弧长评估制动需求。
触发距离应覆盖 d_stop 上界、测量不确定性、安全余量及一个物理步的推进距离。
阈值必须由冻结的停止实验给出；本次 probe 的 0.5 m 位移预算不是生产停止距离。
不能只用到目标的欧氏距离：弧长、末段方向投影和横向位置都必须记录。

若当前仍需大角度转弯，不能套用直线停车数据；需匹配相应状态域的包络或预测。
若首次检测时所需制动空间已经不足，返回明确失败，不延长预算或直接强交 capture。
触发条件采用提前越阈检测，不能要求每步恰好落在一个狭窄距离区间内。

停止包络的条件至少包括 u/w/q、pitch、current、有效限值、固定制动控制律及
PID 记忆的适用范围。仅以速度查询的表不能自动覆盖其余变量。
超出已验证域时返回 UNKNOWN，不外推为安全；v²/(2a) 不作为本模型的安全证明。

## 3. Handoff admissible set

必须同时满足以下条件，并连续保持 N 个物理步；任一失败重置资格计数。
N、入口距离、入口速度和角度误差阈值待停止实验后冻结，不在本合同虚构数值。

- 到目标距离 d_g ≤ d_entry。
- 对地速度 sqrt(u²+w²) ≤ v_entry；不是相对水速度。
- |q| ≤ q_entry，wrapped pitch 与约定捕获入口姿态之差 ≤ theta_entry。
- 不违反现有 capture 实际姿态/角速度 guard 和参考姿态范围；新入口限制不能更宽。
- 剩余弧长足以完成预测制动；预测停止位置不越过约定目标端平面。
- 预测完整制动运动满足原 required_clearance 和侵蚀边界。

目标端平面以最后非零线段切向定义；零长度/singleton 路径无此方向时，不伪造切向，
已到达者走原 settle，否则需独立可验证捕获入口或返回未支持。
已经越过端平面的状态不能被“剩余弧长为零”掩盖；这是保守入口拒绝，不改原成功区域。
入口条件只是必要条件，不能证明后续捕获必然成功或稳态可保持。

## 4. 预测与运行时安全

预测使用相同 Physics、current、限值和制动律，并复制控制器/策略状态，
不能通过试算修改在线积分或实际车辆状态。检查所有预测相邻位置的连续线段，
而非仅检查停止点或当前点到目标的连线；边界与障碍采用与 Navigator 相同的净距。
离散预测需声明 dt、预测时域和空间误差余量；它不是连续非线性运动的形式化保证。
预测未在有限时域内停稳、模型状态无效或缺少可信包络时，不授予安全资格。
运行过程中仍逐步执行原实际运动安全检查，预测不能替代它。

## 5. Timed / LOS 共用语义

两种方法调用同一个 approach 策略、停止模型和 handoff 规则。
Timed 停止推进被替代的 guidance 路径时钟，物理时间和任务预算继续推进；
LOS 的 committed_s 仍只在 PATH_FOLLOWING 更新。Approach 可独立计算几何诊断，
但不得偷偷提交 LOS 进度。原定时参考保留，并单独记录有效 approach 参考。
LOS terminal_ready 是几何提示，不再等价于无条件交接许可；不得伪造时间戳。
所有共同修改应同时作用于两个新实验臂，旧冻结证据不重算回填。

## 6. 失败、日志与验收

建议结构化 reason：STOPPING_ENVELOPE_UNKNOWN、INSUFFICIENT_BRAKING_SPACE、
PREDICTED_UNSAFE、HANDOFF_NOT_ADMISSIBLE、APPROACH_BUDGET_EXHAUSTED。
这些是拟议原因码，当前 NavigationStatus 尚未增加它们。新阶段必须有有限预算，
且包含在原任务总预算中；到时失败，不自动增加任务时限。

记录触发状态、剩余弧长、预测停止位置/净距、包络版本、资格计数、
每步原始/有效参考、commanded/controller/applied control 和交接实际状态。
验收包含：水平/上下斜线不退化、过晚入口拒绝、资格抖动、预测副作用隔离、
危险扫掠拒绝、tight limits 和恒流；不得仅以 detour 一次成功宣告完成。
当前仅交付合同；参数冻结和生产实现另行验收。
