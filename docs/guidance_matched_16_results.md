# Guidance Phase B：冻结候选与 matched 16 场景结果

日期：2026-09-19。状态：本轮冻结、两臂对照、证据核验及失败分类完成；未修改算法或修复失败。

## 1. 冻结身份与证据

候选名：`guidance-v1.0-phase-b-candidate`，不是最终 release。

- Git HEAD：`4e6b08cbbf78aad107cb6cf7ad95e0715f998a82`。
- **工作区有未提交/未跟踪文件，HEAD 不代表 Phase B 的完整内容。** 没有创建 commit/tag。
- `artifacts/guidance-v1.0-phase-b-candidate/candidate.json` 记录 180 个源码/配置/文档文件的 SHA-256。
- 单份 `snapshot.zip` 同时保存源码和 13 个原六场景 evidence 文件，包括 manifest 与 **751 passed** JUnit。
- ZIP SHA-256：`8244b98e84a1c765c0e7aae4d866f645612e84c22aab9558a341fd4b636fe244`。
- 两套原六场景 manifest 均已逐文件核验，运行时源码与这些证据中的哈希一致。
- 原 v0.8、六场景和候选目录均未覆盖。本轮新证据保存在独立目录。

## 2. 实验合同落实情况

采用预先写定的 [对照合同](guidance_matched_16_contract.md)，先完成全部 LOS，再完成全部 timed。
16 场景 × 2 方法 × 2 次重复，共 **64 次闭环运行**。

每个场景只生成一次 A* 和后处理结果，两个方法共享同一缓存结果。逐场景 shared-input hash 一致；
所有状态、参考、控制记录、指标和结果的重复运行检查通过。运行前/每场景前/结束后校验冻结的运行时源码。

共同采用原 16 场景的 **0.5 m/s** 名义速度、post-freeze 70°/75° startup、同一捕获策略、
Physics/current、前馈 PID、限幅/anti-windup、120 s 时限、目标容差与连续安全检查。
保留 saturation 场景原有更紧 surge 限值。未加入 corner slowdown、滤波、平滑或 PID tuning。

LOS 几何参数来自冻结的 `guidance_phase_b.json`；没有因为查看结果而改动。
不要与首轮 detour 0.2 m/s 的成功结果混合比较。

两臂内环相同，但 timed 保留位置/深度外环，LOS 替换为几何 guidance 后直接调用 body-guidance。
因此这是**外层 guidance 系统比较**，不能把全部差异归因于“是否使用时钟”。

## 3. 汇总

| 终止状态 / 诊断 | LOS | Matched timed |
| --- | ---: | ---: |
| SUCCESS | 10 | 9 |
| COLLISION | 1 | 3 |
| OUT_OF_BOUNDS | 2 | 2 |
| CAPTURE_FAILED | 2 | 1 |
| STARTUP_FAILED | 1 | 1 |
| TIMEOUT | 0 | 0 |
| rate-envelope violation（独立诊断，不与上面相加） | 4 | 3 |

LOS 成功数 10/16，timed 为 9/16。唯一新增成功是 saturation 场景；原 timed 成功的 9 个场景
在 LOS 中也成功。LOS 减少了这批已观测碰撞，但更多场景超过角速度诊断阈值，且仍有严重失败。
**不能宣称 LOS 全面优于 timed tracking 或已具备鲁棒安全性。**

这是固定场景描述统计，不是独立随机试验；零海流 current case 重复了 narrow passage。
失败时长是被终止截断的观察长度，不作为更快完成时间参与平均。

## 4. 逐场景对照

| 场景 | LOS | Timed | 最大实际 q 幅值：LOS / timed（rad/s） |
| --- | --- | --- | ---: |
| horizontal | SUCCESS | SUCCESS | 0 / 0 |
| ascending | SUCCESS | SUCCESS | 0.170 / 0.170 |
| descending | SUCCESS | SUCCESS | 0.170 / 0.170 |
| initial_pitch | SUCCESS | SUCCESS | 0.148 / 0.148 |
| initial_speed | SUCCESS | SUCCESS | 0 / 0 |
| saturation | SUCCESS | CAPTURE_FAILED | 0 / 0 |
| narrow_passage | SUCCESS | SUCCESS | 0 / 0 |
| detour | CAPTURE_FAILED | OUT_OF_BOUNDS | 0.986 / 0.809 |
| double_baffle_detour | OUT_OF_BOUNDS | COLLISION | 1.378 / 0.818 |
| edge_forced_detour | OUT_OF_BOUNDS | OUT_OF_BOUNDS | 0.693 / 0.256 |
| u_trap_detour | STARTUP_FAILED | STARTUP_FAILED | 0 / 0 |
| zigzag_maze_detour | COLLISION | COLLISION | 1.008 / 0.851 |
| current +0.00,+0.00 | SUCCESS | SUCCESS | 0 / 0 |
| current +0.10,+0.00 | SUCCESS | SUCCESS | 0 / 0 |
| current -0.10,+0.00 | SUCCESS | SUCCESS | 0 / 0 |
| current +0.00,+0.05 | CAPTURE_FAILED | COLLISION | 0.097 / 0.008 |

## 5. 按失败机制分类：观测与推断分开

### A. 时间参考领先：saturation

Timed 在 t=16 s 结束参考并进入捕获时，实际 x=5.895 m，距目标约 3.105 m，u=0.428 m/s。
最终因捕获尝试次数耗尽失败，仍距目标 1.243 m。LOS 在同样限幅下完成任务。
这直接支持“受限执行速度下，固定时间参考过早结束，而空间进度可避免该失配”的解释。
但两臂外层反馈也不同，不将该现象声称为纯时钟单因素的因果证明。

### B. 末段接管状态不满足捕获 guard：LOS detour

LOS 在 t=30.55 s、目标误差 0.213 m 时尝试进入 capture；此时 q=0.5358 rad/s，
超过原 capture guard 0.35 rad/s，立即返回 CAPTURE_FAILED。该时刻 u≈0.595 m/s，
平移速度约 0.597 m/s。本轮没有放宽 guard，也没有把位置接近解释为成功。

需要区分：**最后一条已完成的运动记录是 PATH_FOLLOWING，失败发生在下一次 capture 入口检查。**
原始 evidence 的 `failure_evidence.phase` 在无新运动步时表示最后观测阶段；派生报告增加
`termination_phase=TERMINAL_CAPTURE`，不篡改原记录。capture_duration=0 不意味着没有发生入口拒绝。

Timed detour 则在 capture 的 BRAKE 阶段越界，最小侵蚀边界裕量 -0.00460 m。
两者是不同失败类型，不能把 LOS 的非碰撞失败直接升级为任务成功。

### C. 海流下捕获制动条件不可达：垂向流 +0.05 m/s

LOS 通过窄通道并进入 capture，随后 **1200 个步长全部为 BRAKE**，耗尽 60 s 捕获预算。
末态 u≈0.00010、w≈-0.0500 m/s，平移速度约 0.0500 m/s，高于 brake_speed=0.03 m/s。
依据现有 policy 的条件，无法进入 ALIGN/MOVE；末态误差增至 3.057 m。
这一 fixture 的日志与源码条件明确指向捕获制动前提和持续海流之间的冲突，而非角速度饱和。

Timed 在 t=6.35 s 的跟随阶段撞到通道障碍，净障碍裕量 -0.00678 m。
LOS 改变了失败所在阶段，但当前仍未完成海流任务。

### D. 几何转弯/执行包络：多折线与边界

- double baffle：timed 在跟随阶段碰撞，观测路径偏差约 1.235 m；LOS 后续在跟随阶段越界，
  边界裕量 -0.01151 m，最大 q=1.378 rad/s。LOS 末态未折返表示的 pitch=-7.505 rad，
  轨迹出现转圈；不能把小 raw 命令增量误认为姿态运动温和。
- edge-forced：两臂都在角点区域越过侵蚀边界，裕量分别 -0.00665 / -0.01834 m。
  路径邻近边界、执行偏差与转向包络值得研究，但本轮没有证明改变哪一个因素必然修复。
- zigzag：两臂都碰撞。LOS 首次碰撞的几何偏差约 0.352 m，timed 约 1.234 m；
  LOS 更接近路径仍不足以通过所需 clearance，且 LOS 最大 q 更大。

三组急转场景的 raw following pitch 最大步变：

| 场景 | LOS（rad） | Timed（rad） |
| --- | ---: | ---: |
| detour | 0.0589 | 1.1172 |
| double baffle | 0.0946 | 1.1071 |
| zigzag | 0.0604 | 1.4289 |

LOS 的参考增量较小，但上述场景实际最大角速度反而更大。因此当前证据不支持
“只加一个 observer/filter 就能解决剩余失败”；应先区分持续参考偏转、姿态响应和可用执行空间。

### E. 共同启动拒绝：U-trap

两臂均在 t=0 因首段航向超出 startup envelope 被拒绝。不是碰撞，也不是成功。
保持现有 guard；没有预对齐初始姿态或只针对 LOS 放宽合同。

### F. Progress 与诊断阈值

本轮 LOS 进度单调违规、progress clamp、switch-limit 触发数均为 0，无 TIMEOUT/progress stall。
没有证据把本轮失败归因于限幅或切段次数上限。仍不能据此证明所有自接近路径均无问题。
rate-envelope violation 按全程 max|q|>0.35 记录，不影响任务终止。

## 6. 可核验交付

- 候选：`artifacts/guidance-v1.0-phase-b-candidate/`
- 32 组结果与共享场景：`artifacts/guidance_matched_16_v1/`，内部记录两次运行一致性。
- 派生报告：`artifacts/guidance_matched_16_report_v1/report.json`，分开记录终止阶段和最后观测阶段。
- 16 场景路径叠图：`artifacts/guidance_matched_16_report_v1/paired_paths.png`
- 全程角速度图：`artifacts/guidance_matched_16_report_v1/rate_envelope.png`
- 新增 6 个比较诊断测试；全仓 **757 passed**，JUnit：`artifacts/guidance_matched_16_tests_v1.xml`。
- 源码入口：`validation/freeze_guidance_candidate.py`、`validation/guidance_matched_comparison.py`、
  `validation/guidance_matched_report.py`。最后一个脚本重新核验所有 evidence hashes 后才生成图表。

复跑必须使用新的输出目录；脚本拒绝覆盖现有证据。artifacts 继续不提交到 GitHub。

## 7. 下一轮建议（本轮未实施）

先以保存的 detour handoff 状态和垂向流 capture 状态建立两组独立诊断，分别研究：
末段接近是否能在不放宽 guard 下进入捕获；持续流下 BRAKE 条件是否与欠驱动可执行性一致。
再研究多折线的全程姿态/角速度和执行空间，不先统一调 PID，也不立即加入平滑。
若以后升级全程 envelope guard，必须对两个方法共同应用并开启新的实验版本。
