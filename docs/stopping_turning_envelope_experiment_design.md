# Step 2 — Stopping / Turning Envelope 实验设计

日期：2026-09-19。状态：**实验设计，未运行、无新测量结果**。
前置：[Contract A](terminal_approach_handoff_contract.md)、
[Contract B](current_terminal_feasibility_contract.md)。
目标是测量固定 LOS / Control v0.5.1 / Physics 的执行能力，不调 PID、
不加平滑/observer、变前视或角点减速，不修改原 16 场景统计。

## 1. 冻结项与测量边界

固定 matched baseline 的源码、LOS 参数、控制前馈、限幅、anti-windup、dt、
Physics 和 current 定义。保存 commit + dirty diff/源码 SHA-256、完整配置及环境版本。
0.35 rad/s 仅为共同诊断线：不提前终止、不作为 Planner 硬拒绝条件。
启动/捕获原有 phase-local guard 不变；本实验测量窗不进入 terminal。
实验结束标为 MEASUREMENT_COMPLETE/CENSORED，不标为 Navigation SUCCESS。

先写并审核可执行配置，再运行；本文件中的矩阵是设计范围而非已测生产包络。
每个配置至少两次重复，比较状态/控制/进度，不把 wall-clock 时间纳入确定性要求。

## 2. 转弯矩阵

主矩阵：名义速度 {0.2, 0.35, 0.5} m/s，转角幅值 {30,45,60,90,120}°，
转向符号 {上转,下转}，world current {(0,0),(0.1,0),(-0.1,0),(0,0.05),(0,-0.05)} m/s。
共 150 配置、两次共 300 次。先跑零流 30 配置作执行器/日志检查，不能依据结果调参。
后续子集和失败也保留；镜像不预设物理/控制对称性。

几何采用水平入段 → 单个拐角 → 足够长的出段，开放空间、无障碍。
路径端点必须远于测量终止位置，保证 LOS terminal_ready 不触发。
初始姿态水平、u=名义速度、w=q=0，控制器记忆初始化方式统一记录；
这是隔离转弯的 cruising fixture，不是把完整任务初始条件偷偷预对齐。
恒流下该初值不必是 trim，必须测量入段 warm-up；未稳定的样本单独标记。

运行配置需预先冻结入/出段长度、开放边界、warm-up 窗口、最大测量时间和稳定带。
路径长度必须容纳 lookahead 提前转弯与出段恢复；不够时记录右删失，不能事后
只对失败样本延长。长入段仍未稳定时报告 ENTRY_NOT_SETTLED，不混入稳定入口包络。
额外完整任务验证仍保留原 startup，不能由 cruising fixture 取代。

## 3. 经验映射与量的定义

条件化映射为 (v,delta_theta,current; 固定入口/控制配置) →
(q_max,e_max,R_executed,d_entry,d_exit,d_clearance)。它不是全状态空间保证。

| 量 | 测量定义 |
| --- | --- |
| q_max | 测量窗内 max(abs(q))；另记 >0.35 的持续时间与比例 |
| e_max / RMS | 实际位置至两条有限路径线段的最短距离；另记有符号当前段 cross-track |
| R_executed | 按实际 world 速度/加速度得到的局部曲率半径分布，不强行拟合单一圆 |
| d_entry | 首次持续偏离入段稳定带的位置到角点的入段有符号投影距离 |
| d_exit | 转弯后首次重新持续进入出段稳定带的位置到角点的出段投影距离 |
| d_clearance | 针对指定 corridor 几何使实测扫掠通过所需的额外空间，不等同通用半径 |

稳定带包含 cross-track、wrapped 相对该段切向 pitch、q 和速度误差，
阈值、连续保持时长及事件插值规则必须在执行配置中预注册。
无偏离事件/未恢复分别记 not_observed / right_censored，不能填零。
段归属采用顺序有限线段逻辑，不用全局最近 waypoint 推断已通过角点。
记录最大 wrapped pitch_ref step、moment saturation 比例/持续时间、进度切段与限幅次数。

曲率使用 kappa=abs(v_x*a_z-v_z*a_x)/||v_ground||^3；半径为 1/kappa。
不能直接用 u/q 代替轨迹曲率：heave/current 会使 course 与 pitch 不一致。
近零速度或近零曲率分别报告 undefined / straight，JSON 用 null 加原因而非 NaN/Inf。
冻结差分方式、低速阈值和有效窗口，报告有效样本比例与半径分位数/最小值。
在预选零流 30°/90°、速度 0.2/0.5 子集用 dt/2 复核数值敏感性；不据此调 PID。

## 4. 空间余量不是一次最大误差数字

保存完整实际扫掠中心线及车辆半径/原 safety_margin，分别报告角点内侧切角、
外侧偏离、入段/出段恢复范围。先在同一开放空间轨迹上离线放置不同宽度的
虚拟 corridor，使用原连续线段碰撞检查，记录首次违规位置和所需额外膨胀量。
clearance sweep 网格在 runner 配置中预先固定；它是后处理反事实，不是新闭环成功率。
再选择预声明的真实障碍场景独立验证，不能以虚拟 corridor 结果声称已避障成功。

未来规划净距应显式写成 vehicle_radius + safety_margin + execution_margin，
避免重复计数。最大偏差只能给出该批轨迹的经验界，未覆盖的速度、流、姿态、
转角和控制记忆必须返回 out_of_domain；不能称为鲁棒安全包络。

## 5. 独立停止矩阵

停止实验与转弯实验分开：先以相同三档速度和五种 current 的稳定直航入口，
施加冻结的现有制动目标/控制律；再加入预注册的 pitch/q/w 入口扰动和
原 detour 保存入口作压力样本。各入口的 PID 记忆必须保留或统一初始化并标注，
不能将不同记忆混成仅依赖速度的一张表。

停稳使用原 capture BRAKE 的对地速度与 q 条件，并要求连续资格时间；
记录停止时间、初始航向投影的最大前移、横向扫掠、总位移/弧长、控制饱和和净距。
最大前移而非最终位移用于评估是否越过目标；单一弧长不能代表横向安全。
固定有限测量预算；垂向流持续漂移、未停稳或越域样本标记删失/不可用，
不能输出虚构有限 d_stop。不同制动控制律的数据使用不同版本，不能混用。
姿态/q/w 扰动网格与测量预算尚待可执行配置冻结，不能现在宣称停止表已完整。

## 6. 证据交付和下一步门槛

runner 未来应输出 config、逐步 history、结果表、summary、图和 manifest；
记录 measurement window、删失原因、入口稳定性、重复一致性和 dt 敏感性。
数据冻结需包含源码/配置/工件校验和，未知域与失败样本也进入版本。
主图为速度×转角的 q、偏差、恢复距离热图，按 current/方向分面；
另画实际转弯扫掠和停止距离分布，不用填色掩盖缺失/删失样本。

完成数据冻结之后才设计可选 execution-aware Planner v0.6c，并保持 geometric A* P0。
P0/P1 固定同一 LOS、控制、终端策略和安全条件进行比较；
terminal policy 修复另起实验，不与 Planner 改动混合。
之后研究 current-aware planning，稳定仿真基线后进入 ROS 2 adapter。
当前交付只完成合同和实验设计，尚无 envelope 数据或新的成功率。
