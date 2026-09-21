# 近期路线：仿真导航基线 → ROS 2 实验室机器人

日期：2026-09-18。用户提出的路线调整正式采用。

1. **当前第一目标**：稳定、可比较的 3-DoF 仿真导航基线。完成 MSS 定向审计、
   finite-segment progress LOS 规范、几何/阶段测试和与 timed tracking 的公平实验。
   所有失败和适用范围进入报告；稳定不等于所有复杂场景必须成功。
2. **第一拓展目标**：通过 ROS 2 建立平台无关导航接口，在实验室实体机器人进行
   路径规划与跟踪实验。先完成消息/坐标/时钟/命令能力适配与回放验证，再接实体平台。
3. **后续拓展**：6-DoF UUV、动态障碍物、更高级 guidance/control；Gazebo/Stonefish
   可作为后期外部验证平台，不是实验室 ROS 2 接口的前置要求。

## 平台无关边界

纯 Python Core/Planner/Guidance 不导入 ROS 消息。ROS 2 adapter 负责任务、路径、
带 frame/timestamp 的状态、能力声明、命令和结果转换；实体机器人使用驱动/底层控制
适配，仿真使用现有 Physics。几何算法可复用，但 x-z UUV pitch 不能简单重命名为
地面机器人 x-y yaw，tau_m 也不能直接发送给速度控制型平台。

平台配置应声明规划平面、坐标轴、可控自由度、速度/力命令能力及有效限值。
任务取消、超时、旧状态处理与停止命令由适配/任务层明确实现。原始日志和实验结果
统一保存方法、平台、单位和时间来源，以便比较；不同平台动力学不宣称等效。

机器人型号、运动约束、ROS 2 版本、驱动话题/动作、定位来源、控制频率和实验场地
尚待实验室信息确认。以上未知项不阻塞当前仿真 guidance 工作，也不在此虚构接口名。
取得平台资料后形成独立 adapter specification，再进行实体接入。

## 本次完成与后续入口

已完成指定八文件的 [源码审计](guidance_source_audit.md) 以及
[LOS/progress specification](guidance_progress_los_specification.md)。
Guidance v1.0 Phase A 已完成纯几何投影、显式进度状态、固定前视 LOS 和
[纯几何验收](guidance_phase_a_validation.md)。可选的
[Phase B 适配](guidance_phase_b_validation.md) 已完成，水平、上斜、下斜、
孤立转弯、低速 detour 和恒流六个小场景通过；PID 与安全设置保持不变。
[固定 16 场景 matched 比较](guidance_matched_16_results.md) 已完成：LOS 10/16、
timed 9/16 成功，角速度诊断超限为 4/16 与 3/16；仍不能宣称鲁棒安全。
本轮只分类失败，没有立即修改 guidance、planner 或 controller。
已将 evaluated baseline 提交为 `66a973c`（未推送），并完成
[两个独立 terminal replay](terminal_handoff_replay_diagnosis.md)：
确认末段制动空间不足的 probe 结果及固定水平捕获在垂向流下的 BRAKE 条件冲突。
已形成 [Terminal Approach / Handoff 合同](terminal_approach_handoff_contract.md) 和
[Current-aware Terminal Feasibility 合同](current_terminal_feasibility_contract.md)，
均为设计文件，尚未实现生产修复。
下一研究顺序为 [Step 2 孤立转弯及停止实验设计](stopping_turning_envelope_experiment_design.md)
→ 冻结可执行矩阵 → 测量并冻结包络数据
→ [Step 2b：零流恢复 v2](zero_current_recovery_v2_design.md)
→ zero-current、fixed-speed、LOS/Control-v0.5.1-specific Planner v0.6c 规范
→ 固定 LOS 的 P0/P1 对照 → 单独实现 terminal policy 改进
→ current-aware planning → 稳定基线后的 ROS 2 adapter。
0.35 rad/s 继续作为诊断线，不自动成为 Planner 的硬拒绝条件。
2026-09-20：已实现 [确定性 measurement runner](envelope_measurement_runner.md)，
完成 pilot、150 配置转弯、106 配置停止和 dt/2 复核，均各重复两次。
见 [结果与删失范围](envelope_measurement_results.md) 和
[数据冻结索引](envelope_dataset_v1.json)。774 项回归通过；尚未开始 Planner v0.6c。
测量完成不等价可靠执行：转弯出段恢复 126/150 右删失，停止 40/106 右删失，
不能直接将这些样本压缩成无条件安全膨胀系数。
v1 正式验收为可信、可复现且带大量删失/域限制的能力测量数据集，
不是可直接交给 Planner 的执行包络。先备份完整 artifacts，再设计并独立执行
30 个零流条件的统一长窗口恢复 v2。
current 数据不扩大窗口、不重标原结果；未来另立 current-conditioned 稳定带研究。
Planner 查询拟采用 SUPPORTED / UNKNOWN / UNSAFE_OBSERVED，不能把删失当有限上界。
完整 artifacts 已完成 [项目外本地备份与校验](artifact_backup_20260920.md)，
保留 1,942 个普通文件及外部目录连接的元数据；尚无异地备份声明。
Step 2b 已完成 [独立 v2 执行与结果](zero_current_recovery_v2_results.md)：
30×2 加 pilot 两次，全部前缀与独立重复分析一致，789 项回归通过。
24 条件恢复并保持到窗末，6 条件恢复右删失；±45°、0.5 m/s 存在再出带，
首次恢复不得直接作为可靠恢复距离。见 [v2 冻结索引](zero_current_recovery_v2_dataset.json)。
已编写 [Planner v0.6c 规范](planner_v0.6c_specification.md)：转向有限的候选路径改进，
不继续扩大能力测量，不要求先构建通用安全包络。下一步实现并验证 P0/P1 零流
0.2 m/s 独立对照，保持同一 LOS/PID，重点评估完整路径碰撞/越界及相邻转弯干扰。
P1 与 [首轮有限对照](planner_paired_v1_results.md) 已完成：12 场景×两臂×两次，
P0 6/12、P1 7/12 成功，碰撞均为 2/12，无越界。只在 detour 改变路径，
其余 11 场景同路径；不得据此宣称通用避碰改善。原 v1/LOS 10/16 未修改。
下一步按这个有限结果决定规划扩展；可并行规划独立 ROS 2 adapter 工作，
不要求先解决 terminal、current 或全部复杂场景。当前未实现 ROS 2 adapter。
2026-09-21：P0/P1 基线提交为 1a8a105；[独立 P2 圆弧对照](planner_curved_v1_results.md)
已完成，P1 7/12、P2 6/12 成功。P2 三例几何拒绝不能算避碰成功，u_trap 启动拒绝；
detour 出现细密采样与现有投影 gate 的兼容性退化，暂不替换 P1，也不因此调 PID。
后续分别处理采样/guidance 兼容性、TerminalApproachPolicy 与 ROS 2 adapter。
不覆盖 geometric A*，不在同一轮混入变前视、减速和 PID 调参。
来源优先级：MSS 算法、UUV Simulator 接口、PVS 控制/reference-model 补充；
Nav2 与 MOOS-IvP 留到后续设计对照。原冻结版本与所有实验数据保持独立。
