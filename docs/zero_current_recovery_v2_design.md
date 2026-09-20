# Step 2b — Zero-current recovery v2 design

日期：2026-09-20。状态：**设计已实现；下文保留预注册依据**。
实施更新：独立 JSON/schema 和 runner 已建立，执行细则见
[v2 executable contract](zero_current_recovery_v2_runner.md)。本设计保留为预注册依据；
实际运行/验收状态以结果报告为准，不将下文原设计时态当作最新进度。
v1 基点为 `23954d9`；[冻结索引](envelope_dataset_v1.json) 不修改。
本阶段不实现 Planner、terminal policy、平滑、减速或 PID tuning。

## 1. 问题与不变量

区分有限 v1 窗口不足、持续振荡/极慢衰减、有界但带外偏差，以及观测到的
发散/持续转圈；不把“穿过角点”当作 stable entry → transient → stable exit。
30 个零流条件：速度 {0.2,0.35,0.5} m/s × 转角 {±30,±45,±60,±90,±120}°。
每项两次，共 60 次；不扩展 current 或重新运行 stopping 矩阵。

继承 v1 的 dt=0.05 s、LOS lookahead/projection、名义速度、PID/前馈、有效限值、
Physics、u/w/q 语义、初始状态及零积分记忆。入口水平段仍为 10 m，角点仍为
(0,-40)，x=-2 m 的入口资格检查不变；保留 startup 不参与的 cruising fixture 身份。
稳定带仍为 cross-track 0.1 m、wrapped pitch 0.08 rad、|q| 0.03 rad/s、
surge 误差 0.05 m/s，连续 1 s。0.35 rad/s 仍只是诊断线。

## 2. 统一延长设计（运行前还需转为独立 JSON/schema）

| 项目 | v1 | v2 拟定值 |
| --- | --- | --- |
| 出段有限长度 | 30 m | 250 m |
| 出段 8 m 结束测量 | 是 | 否；仅保存同位置的比较检查点 |
| 从初始状态计的总测量时限 | 180 s | 360 s，30 个配置完全相同 |
| 发现恢复后提前停止 | v1 仍到测量面 | 不停止，继续到统一时限 |
| 开放场地 | [-60,60]×[-100,20] | [-300,300]×[-340,260] m |

场地扩大仅用于容纳更长直线，是独立开放 fixture 的几何变化；侵蚀 0.1 m 不变。
没有修改任何生产任务的边界/guard，也没有扩大障碍场景的安全空间来提高成功率。
仍逐步检查实际运动段边界；若实际状态进入有限路径最后 10 m、出现 terminal_ready、
越界或数值异常，提前记录明确删失/失败原因，不自动延长路径或预算。
这些值是统一预注册的观测范围，不代表恢复必在此范围内发生。

正常到达 360 s 标为 WINDOW_COMPLETE，不等同 RECOVERED 或 Navigation SUCCESS。
未切入出段记 SEGMENT_NOT_ENTERED；未确认恢复记 RECOVERY_RIGHT_CENSORED。
即便已经恢复，仍观察后续再出带事件。端点只是有限 LOS 几何边界，不调用 terminal。

## 3. 与 v1 的可比性先验检查

仅延长同一出段直线，不能改变前段有效 LOS 目标。实现后应先用冻结 v1 的
30 条零流历史检查，v2 从初始状态直到 v1 对应测量结束时刻的状态、控制、
有效 pitch/surge 与 committed_s 应在声明的浮点误差内匹配。
容差拟定为每个数值分量 absolute 1e-10、relative 1e-10；记录最大误差和首个不匹配。
不要求 distance_to_goal、lambda_raw 或全路径长度相同：出段长度改变会改变这些字段。
比较实际 projection_point/projected_s 和有效命令，不能把段内 lambda 当成弧长。
每个 v2 的两次独立运行仍要求完整历史逐步一致。若前缀检查不通过，停止后续解释，
先检查几何/实现差异；不通过放宽容差或改 PID 消除差异。

## 4. 观测信号、窗口和事件

保留完整 state/control/projection 历史。至少记录：

- 实际位置至有限 polyline 的距离，及相对出段无限直线的 signed cross-track；两者不混用。
- 实际 pitch 对出段切向的 wrapped error、q、surge error；world 速度和航迹角。
- LOS pitch_ref 对出段切向的误差、wrapped reference step、projected_s / committed_s。
- 未限幅请求、Controller 输出、实际施加值、积分记忆与 moment saturation。

按首次 committed segment=1 的时间对齐出段分析，不以全局最近点跨段。
保留原物理时间；新增诊断不推进/修改 progress。切段前数据保留但不混进出段峰值序列。
出段后使用不重叠 10 s 窗口；末段统计使用最后 60 s（只取切段后的有效部分）。
不足 60 s 明示实际长度，短于 20 s 不作尾段分类；末个不足 10 s 的分箱保留并标记。

每个窗及尾段输出 signed mean、RMS、max abs、peak-to-peak、OLS 时间趋势及单位，
同时输出绝对误差趋势。使用误差的幅值趋势判断衰减，不能用负 signed slope 直接称衰减。
角误差跨 ±pi 的窗标为 wrap_crossing；峰值和拟合同时提供连续 unwrap 分支诊断，
不将角度包裹跳变当作物理振荡。不为生成指标而过滤有效控制参考。

恢复事件沿用 v1 的 1 s 规则，保存 first_recovery_time/d_exit；
再记录再出带次数、最长连续带内时间、末 60 s 带内比例及末端是否仍在带内。
“曾恢复”“尾段稳定保持”“直到窗口结束未恢复”是不同结论。

## 5. 局部峰值和现象标签

峰值用于诊断，不新增闭环逻辑。对出段后的 |cross-track|、|pitch error|、|q|
分别找局部极大值，平台取最早最大样本；相邻峰至少间隔 0.5 s，冲突保留较高峰，
同高保留更早峰。记录原始候选、采用的峰及被剔除原因。
采用 prominence 下限分别为 1e-4 m、1e-4 rad、1e-5 rad/s；
prominence 按峰两侧最近局部谷底中较高者计算，边缘缺谷的峰不用于比值。
这些仅是数值诊断分辨率，不是新的稳定带或可执行约束。
输出相邻峰幅值比、峰间时间和最近至多五个有效比值的中位数；不足三个比值
标为 INSUFFICIENT_PEAKS，不硬判衰减。另保存每 10 s 的峰峰值，避免绝对偏置掩盖振荡。

预注册解释标签不互斥，并允许 INCONCLUSIVE：

| 标签 | 所需证据与限制 |
| --- | --- |
| LATE_RECOVERY_OBSERVED | v1 比较检查点之后首次满足原稳定带；需同时报告后续再出带 |
| DECAY_OBSERVED | 最近峰比中位数 <0.95 且至少三比值，并有同向分箱幅值下降；不推断无限时间收敛 |
| SLOW_OR_PERSISTENT_OSCILLATION | 峰比在 [0.95,1.05] 且未稳定保持；必须列具体尾段幅值，不能仅输出标签 |
| BOUNDED_BAND_MISS_OBSERVED | 尾段各 10 s RMS 最大/最小差不超过对应稳定带的 10%，但至少一项持续带外；只称有限窗有界 |
| GROWTH_OBSERVED | 最近峰比 >1.05 且分箱幅值上升；称观测增长而非严格数学发散 |
| CIRCLING_SUSPECTED | 末 60 s 有效 world course 累计有向转角至少 2pi、沿程进展不足 0.5 m；同时给轨迹核验，不只看 pitch |

course 仅对 ground speed>0.03 m/s 的连续片段展开，禁止跨低速缺口累加伪角度。
一个信号衰减、另一个不衰减时分别标注，不强行给整车单一结论。
LOS 参考是否近似稳定也独立报告：末 20 s 最大切向误差≤0.02 rad 且
最大 wrapped step≤0.005 rad 时标记 REFERENCE_NEAR_TANGENT；否则给实际值。
moment saturation 输出比例、总时长和最长连续时长，不只看最大控制量。
上述标签不能区分 PID 与 LOS 的严格因果作用；本轮未做控制器/引导消融。

## 6. current 的隔离与未来 Planner 门槛

不改写 v1 manifest/summary。其 60 个垂向流入口继续保留原 entry_settled=False；
可以在新解释报告中称 ENTRY_OUT_OF_DOMAIN。含流出段的稳定带不满足可解释为
CURRENT_BIASED_RECOVERY_UNKNOWN 或原 RIGHT_CENSORED，但不回填标签到冻结文件。
本轮不扩展含流窗口，也不重定义 current-conditioned equilibrium。

v2 完成后才写 zero-current、fixed-speed、LOS/Control-v0.5.1-specific Planner 规范。
未来三值查询须基于具体入口域和 corridor：SUPPORTED 需要完整恢复与扫掠支持，
UNKNOWN 包括删失/域外/缺样，UNSAFE_OBSERVED 必须有实际违规证据。
长窗口仍不能恢复不自动等于 UNSAFE_OBSERVED；也不对未测条件做插值保证。
P0/P1 主要比较 PATH_FOLLOWING 的碰撞、边界、扫掠、恢复、支持域与饱和；
最终 SUCCESS 保留，但 terminal 失败单独分组，不能掩盖路径执行结果。

## 7. 交付和进入实现条件

后续实现独立 v2 JSON/schema/runner 和几何、峰值、删失、前缀对照测试；
输出 60 次逐步历史、summary、峰序列/分箱、图、manifest 和源代码快照。
新目录拟为 `artifacts/zero_current_recovery_v2/`，必须不存在，禁止覆盖 v1。
开始执行前需确认完整 artifacts 备份已校验，冻结上述设计的可执行配置与分析规则。
本文件记录执行前设计；已完成运行的证据见 [v2 结果报告](zero_current_recovery_v2_results.md)。
没有对 v1 做重新分类。
