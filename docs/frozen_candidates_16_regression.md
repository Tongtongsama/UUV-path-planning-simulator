# 冻结候选：完整 16 场景回归与首次失败分类

运行日期：2026-09-17；中断后证据核验、报告完成：2026-09-18。
本轮没有更改控制器、启动、捕获、局部减速参数，也没有加入 LOS、平滑或 PID tuning。

## 冻结条件与证据

原 `build_cases('all', config/navigation_v08.json)` 的 16 个场景全部保留，
包括初始速度 0.8 m/s、surge 限制 2 N、四组海流和原复杂障碍布局。
采用正式 Control v0.5.1 恢复系数 6 候选、ControlledStartupPolicy、
TerminalCapturePolicy 和现有内部节点前后约 1 m / 0.2 m/s 局部减速。
正常 nominal speed=0.5 m/s，dt=0.05 s，总预算 120 s，所需 clearance=0.30 m。
没有改变初态、规划 clearance 或到达条件。原 A* 与 shortcut 配置保持不变。

每场景重复两次，状态、参考、控制、指标和终止结果一致（不要求墙钟 planning_time 相等）。
**655 passed（16.47 s）**；全部 artifact 与所记录源码 SHA-256 匹配。
原运行已完成，中断后没有假装重新执行整套实验。

证据目录：`artifacts/navigation/frozen_candidates_16_20260917/`。
每案保存初态、场景、全部配置、原路径、轨迹、状态/参考/控制、首次安全违规和诊断；
summary、pytest.log、包版本、源码快照哈希和 manifest 同目录。

```powershell
.venv\Scripts\python.exe -m validation.frozen_candidate_regression --output artifacts/navigation/frozen_candidates_16_new_run
```

## 全部任务结果

| 场景 | 结果 |
| --- | --- |
| horizontal | SUCCESS |
| ascending | SUCCESS |
| descending | SUCCESS |
| initial_pitch | SUCCESS |
| initial_speed | STARTUP_FAILED |
| saturation | CAPTURE_FAILED |
| narrow_passage | SUCCESS |
| detour | SUCCESS |
| double_baffle_detour | COLLISION |
| edge_forced_detour | OUT_OF_BOUNDS |
| u_trap_detour | STARTUP_FAILED |
| zigzag_maze_detour | STARTUP_FAILED |
| current_+0.00_+0.00 | SUCCESS |
| current_+0.10_+0.00 | SUCCESS |
| current_-0.10_+0.00 | SUCCESS |
| current_+0.00_+0.05 | COLLISION |

9/16 成功；2 碰撞、1 越界、3 启动拒绝、1 捕获失败。
这不是 16 个独立随机样本，零流 narrow_passage 在静态/海流组中有重叠。
`evidence_valid` 表示记录与检查通过，`baseline_requirements_met` 仅指原先标记的
must_succeed 子集达标，二者均不代表全部任务成功。启动拒绝不能当作安全通过。

## 首次安全违规（0-based segment/leg）

| 场景 | 阶段标签 | 物理时刻 s | sampled segment / geometric leg | 横向误差 m | reference lead m | pitch 力矩/有效上限 |
| --- | --- | ---: | --- | ---: | ---: | ---: |
| double_baffle | 转角之后 | 33.60 | 28 / 2 | +0.907 | +0.241 | 85.3% |
| edge_forced | 转角区、边界 | 23.55 | 22 / 0 | -0.289 | -0.266 | 57.3% |
| 垂向流 +0.05 | 第一段、窄通道入口 | 6.80 | 12 / 0 | +0.152 | +0.319 | 2.24% |

时刻为失败运动段末端；误差、参考和控制在该段起始时刻测量。
Lead 为正表示参考在当前切向上领先，为负表示实际超前。横向误差带符号。
阶段按参考时钟/进度标记；转角区是节点前后 1 m 弧长，不是因果证明。
特别是 edge_forced 的 leg=0：它仍在第一入段的转角邻域，不能说已执行完转弯。

| 场景 | 起始 reference (x,z) m | 起始 actual (x,z) m | 局部参考 clearance m | 实际弦段 clearance m | 侵蚀边界最小净距离 m |
| --- | --- | --- | ---: | ---: | ---: |
| double_baffle | (5.367,-4.651) | (6.200,-4.220) | 1.128 | 0.29590 | +3.49590 |
| edge_forced | (4.349,-9.306) | (4.285,-9.693) | 0.341 | 0.72603 | -0.00083 |
| 垂向流 +0.05 | (4.125,-5.000) | (3.806,-4.848) | 0.400 | 0.29643 | +3.50631 |

局部参考间隙不同于全路径最小间隙。例如 double_baffle 的全路径最小参考 clearance
只有 0.31623 m，即 16.23 mm 净余量；edge_forced 只有 0.30697 m，即 6.97 mm。
不能把不同位置处的全局误差和间隙直接相减当成碰撞证明。
边界净距离为矩形有符号内距减 0.30 m，检查失败弦段两个端点的最小值；
原边界 unsigned distance before/after 也保存在 JSON，不把边界外的距离误当作安全。

### 解释与限制

1. **Double baffle：横向残差很大，姿态误差已较小。** 失败时实际相对参考 pitch
   误差 -0.05595 rad，内环有效目标误差 -0.03038 rad。车辆大致朝向当前段并不意味
   已回到路径。支持研究空间路径恢复/进度引导，但单个时刻不能排除之前的饱和影响。
2. **Edge-forced：边界空间不足，不能只看障碍 clearance。** 当时内环 pitch error
   -0.01002 rad，并未满输出；参考领先量为负，因此“参考始终跑得太快”不是统一解释。
   需要同时考虑位置偏离、减速后的超前和边界约束，不是终点捕获阶段的失败。
3. **垂向流：窄通道入口横向漂移，没有角点。** 实际相对参考 pitch error
   -0.02568 rad，内环误差 -0.00471 rad，力矩仅 2.24% 上限。当前生成的有效目标
   已被较好跟随，但不足以消除流致横向偏差；仅角点减速不针对该问题。
   这不是证明加大内环 gain 必然有效，也不能预先保证无流 LOS 会抵消海流。

以上是可定位的候选原因，不宣称已完成动态因果分解。

## 非碰撞失败必须单列

- initial_speed：0.15 s 时启动位移约 0.112 m，超出固定 0.10 m 漂移预算，
  实际仍有约 0.696 m/s 速度。当前启动合同不覆盖该高速初态，未放宽预算。
- U-trap / zigzag：t=0 拒绝，首段 pitch 幅度约 81.87° / 66.80°，超出 ±60°
  启动范围。并非实际 pitch-rate 已发散；通用错误文本需结合保存的首段参考判断。
  这两案没有提供任何多转弯执行能力证据。
- saturation：surge 上限为原案例的 2 N，42.10 s 时捕获用尽 6 次尝试；
  终点误差 1.243 m，末速度约 0.029 m/s。全任务饱和占比 45.25%，这是受限执行器
  案例，不能与普通场景混为同一种 guidance 故障。

启动未释放的案例不解释 moving RMSE（空运动阶段可能仍有初始化样本），
也不以零碰撞率、短耗时或低控制能量代理宣称更优。

## 下一步：先限定调研，再设计新 baseline

本轮已完成路线中的冻结回归和剩余失败分类，不继续修改参数。
下一轮短期调研应围绕三个具体问题，而非收集更多 simulator：

1. UUV Simulator 的插值/waypoint/trajectory-point 接口如何分离，如何保证切线、
   速度和时间一致；只审计相关文件及版本/许可证，不整体迁移。
2. Fossen/Python Vehicle Simulator/MSS 的 LOS 空间误差、投影、look-ahead 和
   switching 如何映射到本项目 ENU x-z、nose-up pitch；需明确去掉原位置/depth 外环。
3. 空间进度如何触发终点捕获而不再依赖 timed reference end；路径局部回折和
   segment 跳跃如何避免，以及范围外启动案例如何公平报告。

调研前不声称已审计这些上游功能。本轮未引入第三方代码或依赖。
后续比较必须固定 A* 路径、初态、clearance、内环、Physics、执行器限制与终点条件；
保留 timed、timed+slowdown、progress-based 三种独立方法。
几何路径距离/横向偏离和成功/安全指标用于共同比较；各自的参考 tracking RMSE
不可直接混为同一指标。启动拒绝、预算失败均计入任务结果。

Collision-aware smoothing、基于闭环误差的 execution margin 以及系统 PID tuning
仍按用户路线留待后续；本轮不放宽角度限制、不膨胀地图、不提前实现 LOS。
