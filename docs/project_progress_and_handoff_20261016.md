# UUV 项目进度、实现思路与交接总览

更新日期：2026-10-16。面向：项目作者自查、指导博士讨论、后续 AI agent 接手。

核对基点：`e1eb563`（OCT update），当前分支 `codex/ros2-replay-design`。
本文是进度汇总，不替代各模块 specification，也不重新定义实验验收条件。

## 1. 一页结论

项目已经完成一个**模块化、可复现、具有明确适用范围的 3-DoF UUV 仿真导航与展示版本**。
已经能够从环境与任务出发，生成路径和参考，通过控制器与非线性动力学闭环执行，记录真实运动的碰撞、终止状态、控制量和误差，并从冻结历史生成交互回放与视频。

最重要的系统成果不是“所有场景均成功”，而是建立了可以区分规划、引导、控制、终端策略和路径表示问题的实验体系，并通过独立对照验证了有限改进。

- 原 matched 实验：LOS **10/16**，timed tracking **9/16**，条件为原 0.5 m/s、含流场场景。
- 独立零流、0.2 m/s 的 P0/P1 实验：**6/12 → 7/12**；只有 detour 改变了路径并改善结果，碰撞数没有下降。
- 同一冻结 P1 路径的 TerminalApproach 实验：**7/12 → 10/12**；原七个成功均保留，但耗时增加。
- P2b 独立证明：正确的曲线进度表示可以消除 detour 中密集弦段造成的 LOS 参考退化；没有证明所有困难避碰场景已解决。
- 动态展示已经完成；最新提交另加入 LOTUSim/ROS 2 历史位姿回放代码，但不能据此声称 ROS 端到端运行或实机验证已经完成。

**总体完成度判断：第一个“3DoF 仿真研究＋动态展示”交付点已完成；通用鲁棒自主导航、实机部署和完整 6-DoF 平台尚未完成。** 不采用单一完成百分比，因为上述目标的验收范围不同。

## 2. 本文依据与验证边界

本次核对了当前 Git 状态、目录、主要结果报告、交付说明、回放代码与新 LOTUSim 适配代码。历史统计引用对应冻结报告，不把本次阅读等同于独立复跑全部实验。

本次实际执行：

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_lotusim_replay.py tests/test_replay_export.py -q
```

结果：**13 passed in 1.37s**。覆盖 7 项 LOTUSim 纯映射/输入核验测试与 6 项原离线回放测试；其中调用现有 exporter 检查冻结输入。没有启动 ROS、LOTUSim UI 或重新运行闭环。

历史算法版本的 **847 passed**、展示浏览器验收与视频解码记录，仍属于各自当时的证据。本次没有执行当前 HEAD 全仓测试，不能将历史计数相加后宣称当前全仓通过。

## 3. 系统如何组成

```text
任务 + Environment
        ↓
Planner → 几何 Path → Trajectory / LOS Guidance
                               ↓
                       Controller → Physics
                               ↑        ↓
                         实际状态反馈 ←───┘
                               │
Navigation：调度、startup、approach、capture、安全检查、终止
                               ↓
                     history / metrics / manifests
                               ↓
                    静态报告、离线播放器、ROS 回放适配
```

关键分工：Planner 证明几何路径有效，不保证动力学可执行；Guidance 生成有效跟随目标；Controller 输出控制量，不推进状态；Physics 推进状态，不决定任务；Navigation 编排全流程。展示层读取历史，不重新生成实验结果。

### 3.1 必须保持一致的坐标与物理语义

- 世界坐标 ENU，当前运动平面为 x-z，z 向上，水下位置通常为负 z。
- Body 使用 SNAME：前、右、下；活动自由度为 surge、heave、pitch。
- 正 pitch 为抬头；当前运动学为：

```text
x_dot = u*cos(pitch) + w*sin(pitch)
z_dot = u*sin(pitch) - w*cos(pitch)
pitch_dot = q
```

- `u/w` 是以 body 坐标表达的对地速度；相对水速度在动力学内部扣除海流后计算。不能用相对水速度接近零代替任务停稳。
- 基线欠驱动：可施加 surge force 和 pitch moment，没有直接 heave actuator。
- 参数是 `uuv_3dof_synthetic_v1`，不是已辨识真实机器人，也不能称为完整 REMUS 100 复现。
- ROS FLU 位姿映射需做坐标转换，不能直接照抄 pitch；最新回放使用 `R_y(-pitch)`。三维渲染不意味着动力学升级为 6-DoF。

权威依据：[Physics specification](physics_v0.4_specification.md)、[推导](physics_3dof_derivation.md)。

## 4. 模块完成度与能力边界

| 模块 / 阶段 | 已完成内容 | 状态及尚未覆盖的范围 |
| --- | --- | --- |
| Core v0.2 | Pose、Twist、ControlInput、VehicleState、Path、Trajectory；数据不变量与集成测试 | 基线完成；不是策略容器 |
| Environment v0.3 | x-z 障碍/边界、净距、连续线段检查、NoCurrent/ConstantCurrent、场景加载 | 基线完成；无完整三维海洋物理场 |
| Physics v0.4 | 非线性欠驱动 3-DoF、质量/阻尼/恢复项、海流变换、Euler/RK4、SciPy 对照 | synthetic reference 范围完成；无真实载具辨识、6-DoF、碰撞响应 |
| Control v0.5.1 | 级联 PID、可选实际姿态恢复力矩前馈、合成后限幅、有效限值一致的 anti-windup | 经回归验证；不是广泛工况下的最优控制器 |
| Planner v0.6a/b | 公共请求/结果/空间接口、路径校验、x-z occupancy grid、A*、目标区域语义 | 几何基线完成；不保证执行安全 |
| Planner v0.6c / P1 | 有限 buffer 候选、转角/净距/角点间距等代价排序、完整路径配对执行 | 有限改进验证；不是通用执行包络规划器 |
| Trajectory v0.7 | 去重、shortcut、采样、固定速度时间参数化、终点零速度 | 折线基线完成；角点姿态不保证连续，未做动力学可行性优化 |
| Navigation v0.8 | 固定步长编排、控制与历史记录、连续实际运动检查、结构化失败/成功 | 闭环已完成；两类困难碰撞仍存在 |
| Startup / TerminalCapture | 显式启动、有限终端修正、速度与姿态资格、安全与预算 | 已集成验证；不能修复任意坏入口或不可行稳态 |
| Guidance v1.0 | 有限线段投影、局部搜索、单调进度、固定前视 LOS、显式 capture 交接 | 几何与闭环验证完成；无 adaptive/integral LOS、流补偿或 observer |
| Envelope v1 / recovery v2 | 停止与转弯测量、重复、删失、长窗口恢复与前缀核验 | 数据集完成；不是普遍动态可行性证明 |
| P2 / P2b | 保守圆角几何；独立 primitive 表示实验消除 detour 尺度耦合 | 实验候选；P2b 不属于已验证 10/12 组合 |
| TerminalApproach v1 | 上游有限预测、隔离预测状态、合格入口塑造、实际连续资格交接 | opt-in 有限改进验证；当前零流 LOS 范围 |
| 动态展示 | 五场景、共同物理时间、姿态、同步曲线、播放控制、对照视频 | 冻结历史展示完成；不是实时能力证明 |
| ROS 2 / LOTUSim | 接口设计；已有冻结历史位姿映射与发布代码、纯 Python 测试 | 部分实现；ROS 构建、消息/画面端到端验收待证明 |
| 实机 / 6-DoF | 已明确扩展路线 | 未完成；机器人类型和接口仍需实验室确认 |

## 5. 如何完成：开发与诊断思路

### 5.1 先建立合同，再建立闭环

早期先统一 Core 数据结构和环境查询，把 ROS/Gazebo 类型隔离在外部。Physics 先冻结 ENU/SNAME 符号、降维映射、added-mass 符号、输入 selector 与积分器接口，再用手算方向、解析解、耗散和收敛测试验证。

Planner 先完成公共骨架和几何检查，再加入 A*；目标容差进入搜索语义，而不是只在最终校验时允许。Trajectory 独立负责 Path 到时间参考，Navigation 再把各模块串联，避免一个大脚本同时承担规划、控制和动力学。

这种顺序使后续失败可以归因：几何有效、参考有效和实际执行安全是三个不同命题。

### 5.2 控制问题先看证据，不统一增加 PID gain

斜线和 detour 失败首先区分：力矩已饱和、控制输出不足、定时参考超前、停车与横向残差。恢复力矩前馈有独立实验支持后，才正式加入 v0.5.1，默认关闭，保留旧配置行为。

前馈采用实际姿态；PID 与前馈合成后统一限幅，条件积分使用实际有效限值。Navigation 限值更紧时不能让控制器误以为未饱和。通过显式 BodyGuidance 接口避免新引导与旧位置外环互相覆盖。

相关依据：[控制验收](control_v0.5.1_validation.md)、[终点残差分类](terminal_residual_classification_v051.md)。

### 5.3 从定时跟踪增加空间进度 LOS，而不是删除原基线

定时参考在车辆落后时仍继续前进。因此引入 MSS 思路下的有限线段、基于进度的 LOS：按当前位置投影路径，在有限局部范围内查找，经过顺序 gate 推进 committed progress，再取前视点生成正负号正确的 pitch。

先完成纯几何测试，再接入真实 Controller/Physics；startup 与 capture 保持原职责。保留 timed arm，使用相同路径、速度、控制器和预算做公平对照。原 16 场景结果只支持“LOS 是有价值的另一基线”，不支持全面替代 timed tracking。

来源与边界：[源码审计](guidance_source_audit.md)、[LOS specification](guidance_progress_los_specification.md)、[matched 结果](guidance_matched_16_results.md)。开源主要用于数学和接口参考，不直接移植完整 simulator；具体复制/适配与许可证以 [reuse register](open_source_reuse.md) 为准。

### 5.4 用独立测量认识能力边界

Envelope v1 共 588 次运行，包含停止、转弯、重复和选定 dt/2 检查。150 个转弯条件均穿过测量结束面，但只有 24 个在窗口内确认恢复，126 个恢复距离右删失。

这说明“通过角点”不等于“稳定进入下一段”。0.35 rad/s 继续作为诊断线，不能突然成为全任务硬拒绝阈值。停止距离也依赖姿态、w/q、积分记忆、海流和控制律，不能仅按速度查表。

随后 v2 对全部 30 个零流条件统一延长至 360 s，各重复两次，另有两次 pilot。24/30 在观察窗口内恢复并保持，6 个 0.5 m/s 大转角仍未确认恢复；两例有再出带。保留 first recovery 与 last reentry，不能将首次短暂恢复当作可靠恢复距离。

依据：[v1 结果](envelope_measurement_results.md)、[v2 结果](zero_current_recovery_v2_results.md)。删失是“在给定观察窗未确认”，不是已证明永远不可执行。

### 5.5 P1：有限改变路径，接受有限或阴性结果

P1 在原几何 A* 之上生成有限候选，按总代价最小排序，考虑路径长度、转角、净距和角点间距。它不是对所有曲率路径的全局最优搜索，也不将经验代价包装为安全认证。

P0/P1 均为零流 0.2 m/s、相同 LOS/PID/Physics 和终止条件；从真实初态连续执行，不在角点重置状态或积分。12 场景两臂各两次，共 48 次。

只有 detour 选中不同路径：转角由 5 降到 2，路径长约 10.071→10.485 m，实际最小障碍净余量约 0.3666→0.8643 m，TIMEOUT→SUCCESS。其余路径相同，因此本轮不能说明困难碰撞已改善。

依据：[P0/P1 报告](planner_paired_v1_results.md)。

### 5.6 P2/P2b：几何平滑不等于引导参考平滑

P2 尝试碰撞检查后的圆弧 fillet，不使用未经安全验证的普通 spline。部分路径因相邻切线区间重叠或启动方向不合格被拒绝。这只说明该候选不能放入指定圆弧，不证明整个场景不可行。

detour 的密集圆弧弦段缩小了 endpoint gate，造成进度停滞、LOS 突跳及力矩饱和。P2b 因此分开 dense safety geometry 与 primitive guidance progress，在不改 PID 的情况下验证表示问题。

| detour 指标 | 原 P2 | P2b primitive |
| --- | ---: | ---: |
| 最大 pitch reference step | 0.8129 rad | 0.0128 rad |
| 最大实际绝对 q | 1.3875 rad/s | 0.2846 rad/s |
| 力矩饱和时间 | 7.35 s | 0 s |
| 最长进度平台 | 6.35 s | 0 s |
| 完成时间 | 72.45 s | 63.90 s |

三种密度的实际历史相同支持本例采样兼容性。P1 的 12 场景回归只是未启用新适配器时行为不变，不能说 primitive 已通过全部 12 场景。

P2b 保留在 `c9d809e`，其报告可只读查看：

```powershell
git show c9d809e:docs/p2b_representation_results.md
```

### 5.7 TerminalApproach：在上游塑造入口，而不是放宽 capture

double_baffle 原失败始终处于 PATH_FOLLOWING，terminal_ready 为零、capture 零步；最近距离 0.2678 m 仍大于 0.25 m，且姿态、速度和角速度不适合直接交接。因此没有调宽门限，而是提前预测完整接近过程。

策略复制当前状态、控制器记忆与导航进度，在有限时域内尝试预冻结低速/制动候选；每条预测运动段检查安全。预测副本不推进真实积分。有限预算找不到候选返回 UNKNOWN，不宣称物理不可行。执行后仍须实际状态连续满足原几何与 capture 入口资格才交接。

double_baffle 在 71 s 接管，98.25 s 合格交接，98.30 s 成功。capture 以 ORIGINAL_SETTLE 收尾，没有额外修正位置；主要改善来自 approach。两次碰撞在评估区之前发生，预测步数为零，因此与本轮末段修复无关。

依据：[实现合同](terminal_approach_v1.md)、[配对报告](terminal_approach_v1_results.md)。

## 6. 实验结果不能合并成一个成功率

| 实验 | 条件 | 结果 | 可支持的结论 |
| --- | --- | --- | --- |
| Timed / LOS | 16 场景，原 0.5 m/s，含海流 | 9/16 与 10/16 | 不同参考推进方式的对照 |
| P0 / P1 | 12 零流场景，两臂 0.2 m/s | 6/12 与 7/12 | 有限路径候选改善一个场景 |
| P2b | 固定解析 detour，表示/密度变化 | primitive 明显降低参考退化 | 接口尺度耦合修复，不是全场景避碰证明 |
| P1 / Approach | 同一 P1 路径、12 零流场景 | 7/12 与 10/12 | 合格终端入口改善任务完成率 |

TerminalApproach 两臂最终状态：P1 为 7 SUCCESS、2 CAPTURE_FAILED、1 TIMEOUT、2 COLLISION；启用后为 10 SUCCESS、2 COLLISION。原七个成功保留，但通常增加约 4.65–5.70 s。

所有这些计数按场景条件统计，不将两次重复当成两倍独立场景。**P2b＋Approach 未做组合验证，不能计入 10/12。**

## 7. 已交付的展示与最新 ROS 进度

### 7.1 离线交互展示：已完成

入口：[播放器](../artifacts/replay_display_v1/index.html)，[对照视频](../artifacts/replay_display_v1/double_baffle_comparison_4x.webm)，[说明](offline_replay_v1.md)。

五个场景：horizontal、detour、double_baffle、edge_forced_detour、zigzag。两臂共用物理时间，先结束者保持终态；动画插值只用于显示，原始记录不被覆盖。碰撞显示检测步区间，不虚构精确接触时间。艇形是姿态示意，净余量以日志为准。

推荐向导师演示：先 horizontal 解释闭环，再 detour 解释路径与跟踪，然后 double_baffle 解释上游末段改进，最后主动展示两个碰撞作为能力边界。

### 7.2 LOTUSim / ROS 2：已有代码，但验收层级不同

当前 `e1eb563` 新增 `ros2/lotusim_replay/`：读取经过已有 exporter 核验的历史，转换 ENU/FRD→FLU 位姿，向 LOTUSim renderer 发布对象与位姿。支持场景选择、两臂、播放速度和显示用侧向偏移；不输出实际控制指令，不重新跑算法。

本次 7 项相关纯 Python 测试通过，不代表 `colcon` 构建、DDS 消息兼容或 LOTUSim 三维画面已验收。现有代码面向渲染：消息使用节点当前时间戳，不等同于设计文档提出的完整 source-time `/clock` 与 rosbag 往返合同；该完整平台无关回放接口仍需独立落实。

旧 [ROS 接口设计](ros2_interface_replay_design_v1.md) 中“尚未实现”是当时状态；最新应理解为“通用设计仍待完整落实，特定 LOTUSim 渲染适配已有初版”。不篡改历史冻结说明来制造事后完成。

运行说明：[LOTUSim 回放](lotusim_frozen_replay_v1.md)。尚未确认实验室机器人类型、定位来源、命令层级、执行频率与安全接口，不能把当前推力输出直接假定为 `cmd_vel`。

## 8. 重要版本与证据位置

| 标识 | 含义 |
| --- | --- |
| `23954d9` | stopping / turning envelope v1 runner |
| `1a8a105` | P0/P1 paired v1 冻结 |
| `5c86a28` | P2 圆角对照与阴性结果 |
| `c9d809e` | 独立分支 P2b 表示实验 |
| `f6b6d38` | 已评估 TerminalApproach 算法版本 |
| `dbd4e5f` | 首个 3DoF 交付点整理 |
| `1cb91d7` | 动态展示与备份记录 |
| `766b69d` | 展示冻结与 ROS 只读回放设计 |
| `e1eb563` | LOTUSim 历史回放初版；本文核对基点 |

主要入口：

- [首个交付点及复现命令](3dof_checkpoint_1.md)
- [仿真＋展示冻结说明](3dof_simulation_display_freeze.md)
- [控制验证](control_v0.5.1_validation.md)、[Physics 验证](physics_v0.4_validation.md)
- [P1 结果及配置范围](planner_paired_v1_results.md)
- [TerminalApproach dataset 索引](terminal_approach_v1_dataset.json)
- [备份说明](artifact_backup_20260929_checkpoint1.md)

源码、配置、报告和 dataset 索引在 Git 中；原始逐步 history、图、视频及 manifest 位于被忽略的 `artifacts/`。索引不是原始数据备份。已记录的完整本地备份为 `D:\UUV_Backups\uuv_artifacts_20260929_checkpoint1`（2,486 文件，约 2.25 GB），展示备份为 `D:\UUV_Backups\uuv_replay_display_20260930_v1`。这些是历史核验记录；本次没有再次逐文件核验 D 盘，也不把 D 盘副本称为异地备份。

## 9. 自查与复现方法

先从仓库根目录确认版本和输入，不覆盖冻结目录：

```powershell
git status --short --branch
git rev-parse HEAD
.\.venv\Scripts\python.exe -m pytest tests/test_replay_export.py tests/test_lotusim_replay.py -q
.\.venv\Scripts\python.exe -m ros2.lotusim_replay.lotusim_replay.player --repo-root . --validate-only
```

最后一条为无 ROS 的输入核验入口，不是 ROS 发布验收。完整算法复现请按 [checkpoint guide](3dof_checkpoint_1.md) 的先导→完整实验顺序，使用新的输出目录并恢复所需冻结 P1 输入。缺失 artifacts 应报告缺失，不能悄悄重新生成替代历史。

依赖分层：`requirements.txt` 为基础依赖；`requirements-validation.txt` 包含测试及 SciPy；`requirements-visualization.txt` 包含绘图。依赖范围不等于精确环境锁定，复现时还需记录 Python、安装版本、源码哈希和配置。不要为编写总结而升级运行环境。

自查清单：

- [ ] 能说明每一个成功率对应的速度、流场、场景集、路径和控制配置。
- [ ] 能从 summary 定位到 run history、manifest 和源码身份。
- [ ] 能展示失败案例，而非只筛选成功图片。
- [ ] 能区分 reference、effective guidance、requested control、applied control。
- [ ] 能解释最小净余量、q 诊断线、删失和 UNKNOWN 的含义。
- [ ] 在另一目录恢复证据后能打开播放器；完整实验复跑使用新目录。
- [ ] ROS 纯映射测试、ROS 发布验证、实机验证分别记账。

## 10. 尚未完成与不应宣称的能力

1. **鲁棒避碰未完成。** edge_forced 和 zigzag 仍碰撞；double_baffle 最小净余量仅 7.2 mm，不足以支持模型误差下的安全保证。
2. **通用转弯可执行包络未完成。** 测量存在适用域、删失和多角点相互作用；未知入口不能静默套用水平稳定入口表。
3. **current-aware terminal 未完成。** 欠驱动车辆在受限姿态下未必能实现对地静止 trim；已有合同不等于已实现生产策略。
4. **预测实时性未证明。** 同模型预测与实际一致只支持实现一致性；累计预测开销不能当作满足每步实时预算。
5. **真实载具验证未完成。** 没有真实参数辨识、传感噪声/时延建模和硬件闭环证据。
6. **6-DoF 未完成。** 需要完整坐标与姿态表示、六维动力学/水动力、三维碰撞、执行器与输入分配、三维规划及对应验证，不是只扩展数组长度。
7. **平台无关 ROS adapter 尚未完整验收。** 需明确状态所有权、时间与 frame、日志往返、watchdog、暂停/复位；真实命令接入应晚于只读回放。

## 11. 下一阶段建议与交接规则

当前不再为已冻结提交点扩大 PID/平滑实验。近期可在保持算法冻结的前提下，完成 Jazzy/LOTUSim 回放环境构建及端到端消息和画面验收；再根据实验室资料选择平台适配。困难路径改进、current terminal 和真实控制接入应各自独立立项，不混成一次修改。

指导博士讨论时优先确认：

1. 当前研究成果是否足以作为 dissertation 的仿真基线与因果对照章节。
2. 论文核心贡献如何聚焦于模块化验证、执行证据反馈规划、路径表示或终端入口塑造，避免宣称提出全新通用控制算法。
3. 实验室机器人类型、ROS Jazzy 环境、输入是速度还是推力、定位与安全停止接口。
4. 下一阶段验收是日志回放、仿真桥接还是实机路径跟踪；分别需要什么证据。

### 给后续 AI agent 的工作合同

- 先读本文、checkpoint guide 与当前任务相关 specification；读取 `git status`，保留用户未提交修改。
- 当前正文基点为 `e1eb563`，后续 HEAD 变化后必须重新核查，不将本文当成永久实时状态。
- 不改原 LOS 10/16，不混算实验，不将 P2b 合并效果计入 approach 10/12。
- 不为提高成功率放宽安全条件、延长任务预算、重置角点状态或悄改初态。
- 保护冻结 artifacts；所有新实验使用独立配置、输出目录和 manifest。
- P2b 是独立分支成果；若要合并或与 approach 组合，先获授权，再执行专门回归。
- `UNKNOWN`、`RIGHT_CENSORED`、`UNSMOOTHABLE` 不等于物理不可行。
- 不将 .35 rad/s 诊断线升级为全程 guard，不在无新证据时开始 PID tuning。
- ROS/渲染适配不应导入或改写核心算法，不把展示插值样本写回研究数据。
- 报告中明确写“本次实际运行了什么”和“哪些结果仅引用历史证据”。

## 12. 可直接用于进度汇报的摘要

本项目已建立纯 Python、ENU 垂直平面下的欠驱动非线性 3-DoF UUV 导航研究平台，完成环境、A* 路径规划、时间轨迹与 LOS 引导、PID 控制、动力学、安全终止、记录及动态展示的模块化闭环。通过匹配条件的对照实验，识别了定时参考超前、转弯恢复缓慢、密集曲线表示与 progress gate 耦合、末段交接过晚等不同问题。有限规划候选改善了 detour 路径；独立 P2b 表示实验在不调 PID 的情况下改善了曲线参考连续性；TerminalApproach 在同一零流低速 P1 路径集上将完成率从 7/12 提升到 10/12，同时保留两个碰撞案例和时间开销等限制。当前成果定位为可复现、适用域明确的仿真研究版本，不宣称通用鲁棒安全或真实载具验证。动态回放已完成，ROS 2 Jazzy 的 LOTUSim 历史回放适配已有初版，下一步重点是端到端接口验收与实验室平台信息确认。
