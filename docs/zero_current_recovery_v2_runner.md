# Step 2b executable contract — v2

日期：2026-09-20。依据 [设计](zero_current_recovery_v2_design.md)。
独立配置：`config/zero_current_recovery_v2.json`；exact-value JSON Schema：
`config/zero_current_recovery_v2.schema.json`。v1 配置及工件不改写。
runner：`validation/zero_current_recovery.py`；不导入或调用 Navigator / terminal policy。

## 冻结范围

零流 30 条件 × 2 次，dt=0.05 s、360 s、出段 250 m、统一扩大开放场地。
控制器/Physics/LOS 及稳定带继承 v1。末段剩余 10 m、terminal_ready、边界或数值
异常会删失；首次恢复不会结束运行。预声明 pilot 是 `turn_c0_v0.5_a90`，也重复两次。
先完成该样本、核验日志并审查图，再运行全部 30 条件，包含 pilot 条件而非排除它。

## 前缀合同（全局解释门槛）

读取冻结 v1 的每次 repeat history。比较范围严格使用原 `states` / `steps`
长度、零基 step index、原时间戳，直到原 MEASUREMENT_COMPLETE 事件为止。
校验 state_count=step_count+1、结束时间与原 summary 一致、结束时处于出段且
穿过原 8 m 测量面；不是按 v2 再检测一次过线来决定比较长度。

比较全部原状态、每步时间/elapsed、有效目标、施加控制、完整 controller diagnostics
（包括 requested/output、有效限值、前后积分记忆）、world velocity 和 projection。
初始记忆、原结束步的最终积分也单独比较。数值 atol=rtol=1e-10；
布尔、字符串和结构严格相等；保存最大绝对数值差、原结束步号、时间和事件。
不在容差内时输出 PREFIX_FAILURE.json 并停止；没有合格 summary/manifest，
整个数据集不得解释，不允许只删除失败配置。

逐步字段白名单只有：

- `projection.mu`、`projection.lambda_raw`，且仅当原 projected_segment_index=1。
  两者按出段总长度归一化，长度改变必然不同；投影点和弧长仍须一致。
- `boundary_margin`，因为开放测量场地整体扩大；实时安全检查仍然执行。

顶层新路径端点/长度、测量 status、分析字段不是旧历史步的比较对象；
distance_to_goal 未存入 v1 逐步格式，因此不虚构其一致性证明。
任何其他历史键变化都会失败，不能通过泛化忽略一个 projection/diagnostics 对象。

## 每次运行独立分析

每个 repeat 独立产生 history、analysis 和标签；再比较完整有限 JSON 的 SHA-256。
history 和 analysis 均相同时才宣称 deterministic。`repeat_id` 保存在结果封装层，
不参与数值比较；两次历史和分析文件都保留，不用一个平均结果替代。

恢复 run 用样本首尾时间差，非“样本数 × dt”；资格需连续至少 1 s。
恢复分析的 segment 取当前状态到达时已提交的段号，与 v1 的步序保持一致。
保存以下不同信息：

- first_recovery_d_exit、first_recovery_time、first_recovery_confirmed_time；
- last_reentry_d_exit：最后任意一次入带的起点（可能未满足 1 s）；
- last_qualified_reentry_d_exit：最后满足 1 s 的入带起点；
- reexit_count：首次合格恢复开始之后，所有再次出带事件数量；
- longest_in_band_duration、完整 in_band_intervals；
- stable_through_observed_end：最终连续带内段覆盖最后 60 s；
- stable_through_window_end：还必须正常完成预注册的 360 s 窗口。

稳定到窗末不是无限时间稳定性或 Planner SUPPORTED 证明。首次恢复距离只是诊断量，
若再次出带或尾段不稳，不得作为可连接转弯的可靠恢复距离。

## 分箱、峰值和标签

出段后 10 s 半开分箱；最后不足 10 s 的箱标 partial。
末 60 s 统计含边界样本，固定 1e-9 s 对齐容差仅消除累积浮点时间误差。
统计分别给 cross-track、wrapped pitch error、q、surge 的均值/RMS/幅值/趋势；
pitch wrap crossing 另标并提供 unwrap 诊断，不从包裹跳变推断振荡。

峰值采用绝对信号、最早平台点、最近双侧谷底 prominence 和 0.5 s 分离规则。
候选与拒绝原因全部保存，最近峰比仅为诊断启发式；正负非对称振荡的交替峰、
微小分箱差和窗口相位可能使单个 DECAY 标签与长期 RMS 现象不一致。
因此原始历史、分箱、尾段统计是权威；多标签只是索引，没有单一 classification。
不据标签修改控制、延长预算或筛除失败结果，也不声称通过这轮实验确定 PID/LOS 因果。

## 运行与证据

```powershell
.\.venv\Scripts\python.exe -m validation.zero_current_recovery --stage pilot --output artifacts/zero_current_recovery_v2_pilot_verified
.\.venv\Scripts\python.exe -m validation.zero_current_recovery --stage full --reviewed-pilot artifacts/zero_current_recovery_v2_pilot_verified --output artifacts/zero_current_recovery_v2
```

目录必须不存在。pilot/full 要求相同源码哈希；启动和结束均核对源码不变。
启动时校验 v1 全部冻结工件及本实验使用的旧核心源码；不将新 runner 添加到旧 manifest。
每个条件保留两份 history.gz、两份独立 analysis、轨迹图、分箱图。
最终 manifest 绑定输出、源码/测试/配置快照及上游 v1 manifest。
初始 `zero_current_recovery_v2_pilot` 保留为未采用 pilot：仅修正尾段时间浮点对齐和
图形坐标布局后，以相同动力学配置重跑到 `_pilot_verified`；未调整稳定带或物理参数。
