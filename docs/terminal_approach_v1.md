# Terminal Approach v1：有限零流候选实验

2026-09-29。opt-in，不替换默认 P1；与 P2b 独立。不修改 PID、前馈、
anti-windup、capture guard、任务时限、成功条件或原路径。不是生产安全保证。

## 运行前冻结

权威参数：`config/terminal_approach_v1.json`。同一冻结 P1 路径，0.2 m/s 原跟随，
仅 approach 的接近动作采用候选速度。不是 Planner 速度公平比较，而是末段策略单因素对照。
剩余弧长 ≤4 m 起评估，每 2 s 一次，最多 16 次；每次最多 6 个候选，
速度按 0.15、0.10、0.05 m/s，制动距离按 0.35、0.20 m 组成固定顺序。
使用与真实 Physics 相同的 0.05 s 步长，预测最多 30 s，并截断到剩余任务预算。
最多 57,600 次预测物理步/任务；记录实际步数和壁钟开销，不声称实时可运行。

每个候选模拟完整动作：低速有限线段 LOS → 最后一段且到目标小于候选距离时
不可逆制动 → 真实几何 readiness 与入口资格连续满足三步。
制动目标沿最终切向，surge 采用现有 capture 的有界反向制动律；这是待验证的
候选动作，不把距离常数声称为实测停止包络。角点速度/半径及 PID 均未重新调参。

入口要求原来的末段、剩余弧长和实际目标距离条件，同时对地速度 ≤0.03 m/s、
|q|≤0.03 rad/s、相对最终切向姿态误差 ≤0.15 rad；实际和参考姿态满足原 capture guard。
预测每条运动段以 required clearance +0.005 m 检查边界及障碍；制动不能越过最终目标平面。
5 mm 是声明的实验预测裕量，不是鲁棒误差界，也不改变真实安全合同。

## 状态所有权与失败

复制完整 controller（包括积分）、Physics、progress，绝不在真实对象上做试算。
PATH_FOLLOWING 未接管时照旧；UNKNOWN 暂时保持原路径跟随，下次定时评估。
预算耗尽或进度耗尽仍无候选时返回 APPROACH_FAILED/UNKNOWN_SEARCH_EXHAUSTED，
不解释为物理不可行，不强行进入 capture。

找到第一个完整可行预测才进入 TERMINAL_APPROACH。此后不返回 PATH_FOLLOWING，
原 progress 冻结，approach 私有 cursor 推进；日志保留两者语义，不伪造时间参考。
执行与预测调用同一个反馈核，真实控制器照常积分。逐步状态与预测偏差超过
1e-6（六维各分量绝对差的实验一致性阈值）即结构化失败。
这只是同模型零流确定性复现检查，不是面对模型误差的容错方案。
实际状态重新累计三步资格才交接，预测合格不代替实际检查。capture 代码不变。
接管后仍使用原来的全任务成功判据，保持时间不会因 phase 切换重置。
若交接前已经达到原成功条件，不能把成功解释为 capture 修正动作得到验证；
本轮报告区分合格交接、ORIGINAL_SETTLE 和真正执行 capture 修正。
现有 `final_path_progress` / pitch-reference-step 指标只覆盖原 LOS cursor/commands，
不代表 approach 私有进度或全阶段参考跳变，不能直接用于跨臂优劣声明。

## 首轮门槛与证据

double_baffle、detour、horizontal，每臂两次；旧 P1 完整 history 必须匹配存档。
仅三个新策略场景均成功才扩大到 12 场景；否则保留阴性结果，不看结果后调参数。
输出逐步原参考/有效指令、首次 approach 状态、每次预测原因、积分入口、预测/实际
交接状态、净距、饱和时间、物理计算步数及壁钟开销。重复一致性不含壁钟时间。

复现：`.venv/Scripts/python.exe -m validation.terminal_approach_comparison --output artifacts/<fresh-dir>`。
本版只支持零流 LOS；timed adapter、模型误差鲁棒性和持续流 trim 不在本轮范围。
