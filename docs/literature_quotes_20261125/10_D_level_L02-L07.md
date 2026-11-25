# D 级取证 · 第一批（L02–L07）

2026-11-25。取证方式：从 `ref/` 里的原版 PDF 抽取文本层，逐条定位原句。

**转录约定**：这些 PDF 的文本层是 OCR 产物，存在两类瑕疵——(a) 连字 `fi/ff` 被写成 `Þ/ß`；(b) 断词处丢字符（如 `s-ory` 应为 `satisfactory`）。我在引文中用 `[ ]` 标注补字，其余一仍其旧，**不改写句子**。

---

## Q02 · 仍然是缺口的三条

| 条目 | 状态 | 原因 |
|---|---|---|
| Q02-1 6-DOF 矢量式 | ✗ 未取证 | `L02-1_fossen2011handbook.pdf` 是**扫描版**：54 MB 只抽出 3.9 万字符（≈ 前 30 页），第 2–8 章无文本层 |
| Q02-2 相对速度 ν_r | ✗ 未取证 | 同上；Ch8 §8.3“3-D Irrotational Ocean Current Model”正好落在扫描页内 |
| Q02-3 附加质量取常数 | ✗ 未取证 | 同上。旁证：书第 1 章有一句"Simulation models where the fluid-memory effects are included due to frequency-dependent added mass … typically consist of 50–200 ODEs"，可支撑“常数附加质量是简化”，但不是要的那句 |

> 这三条只能翻纸质书或换一个有文本层的版本。**不要**用讲义 PDF 代替：讲义是子集字体编码，抽出来是乱码。

---

## Q03 Healey & Lienard 1993

### Q03-1 6-DOF 模型 + 滑模自动驾驶仪

> “A six degree of freedom model for the maneuvering of an underwater vehicle is used and a sliding mode autopilot is designed for the combined functions.”

- 出处：`L03-1_healey1993multivariable.pdf` 摘要（OCR 丢了 “steering, diving, and speed control”，只剩 “combined functions”）
- 直译：采用一个用于水下航行器机动的六自由度模型，并设计了滑模自动驾驶仪以完成联合功能。
- **与笔记的差异**：笔记写“覆盖操舵、下潜与速度”，这是被 OCR 截掉的原文内容；正文第 I 节有更完整的表述，引用时以正文为准。

### Q03-2 解耦建模是显式假设

> “This paper shows that a multivariable sliding mode autopilot based on state feedback, designed assuming decoupled modeling, is quite s[atis]factory for the combined speed, steering, and diving response of a slow speed AUV.”

- 出处：同上，摘要第 4 句
- 直译：本文表明，一种基于状态反馈、**在设计时假定解耦建模**的多变量滑模自动驾驶仪，对低速 AUV 的速度、操舵与下潜响应而言相当令人满意。
- **可用性**：`designed assuming decoupled modeling` 是全台账里唯一逐字证明“解耦是假设而非事实”的句子，现在拿到原文了。注意限定：`slow speed AUV`——低速是解耦成立的条件之一。

### Q03-3 LOS 在原文中的角色

> “Waypoint acquisition based on line of sight guidance is used to achieve path tracking.”

- 出处：同上，摘要末句（原文无连字符：`line of sight`）
- 直译：基于视线（LOS）引导的航路点捕获被用于实现路径跟踪。

---

## Q04 Aguiar & Hespanha 2007

### Q04-1 可行参考轨迹的弱条件

> “The desired trajectory does not need to be of a particular type (e.g., trimming trajectories) and can be any sufÞciently smooth bounded curve parameterized by time.”

- 出处：`L04-1_aguiar2007trajectory.pdf` 摘要（`Þ` = `fi`）
- 直译：期望轨迹不必属于某一特定类型（如配平轨迹），可以是任意足够光滑、有界、以时间为参数的曲线。
- **与笔记的差异**：笔记原写“需二阶可微且满足可行性条件”——原文只说 `sufficiently smooth`，没有给阶数。引用时不要写“二阶可微”。

### Q04-2 自适应切换监督 + Lyapunov 律

> “For a general class of vehicles moving in either 2- or 3-D space, we demonstrate how adaptive switching supervisory control can be combined with a nonlinear Lyapunov-based tracking control law …”

- 出处：同上，摘要第 2 句
- 直译：对在 2 维或 3 维空间中运动的一般类别载具，我们演示了自适应切换监督控制如何与基于 Lyapunov 的非线性跟踪控制律相结合……
- **可用性**：这句同时钉死了两件事——适用范围是“2-D **或** 3-D 的一般类”，机制是“监督切换 + Lyapunov 律”。笔记此前把自由度写成 3H，已据此改正。

---

## Q05 Johansen & Fossen 2013 / Fossen & Johansen 2006

### Q05-1 可达集与“投影到边界”

> “Some constrained control allocation methods are based on some scaling of the unconstrained optimal control allocation, such that the resulting control allocation is projected onto the boundary of the set of attainable generalized forces. … Here, the set of attainable generalized forces is denoted A and is the set of vectors … when the constrained optimization problem (e.g. (8)) has a feasible solution.”

- 出处：`L05-1_johansen2013control.pdf` §2.2.2 “Direct allocation”
- 直译：一些带约束的分配方法先求解无约束最优分配，再做缩放，使分配结果被**投影到可达广义力集合的边界上**。……可达广义力集合记作 A，是使约束优化问题有可行解的那些向量构成的集合。
- **可用性**：这就是“饱和 = 指令落在可达集之外”的原始出处。注意原文措辞是“投影到边界”（scaling），不是“修改期望力”——写论文时应沿用原文措辞。

### Q05-2 海上载具的分配是带约束的优化

> “Control allocation problems for marine vessels can be formulated as optimization problems, where the objective typically is to minimize the use of control effort (or power) subject to actuator rate and position constraints, power constraints as well as other operational constraints.”

- 出处：`L05-2_fossen2006survey.pdf` 摘要首句
- 直译：船舶的控制分配问题可表述为优化问题，目标通常是使控制消耗（或功率）最小，约束包括执行器的速率与位置约束、功率约束以及其他运行约束。
- **可用性**：这句直接支撑“位置**与速率**双重约束”，比笔记里只写“箱式约束”更准确。

---

## Q06 抗饱和

### Q06-1 抗饱和的两步结构

> “In anti-windup compensation, a linear controller which does not explicitly take into account the saturation constraints is Þrst designed, usually using standard linear design tools. Then, after this controller has been designed, a so-called anti-windup compensator is designed to handle the saturation constraints.”

- 出处：`L06-1_tarbouriech2009antiwindup.pdf` 引言（`Þrst` = `first`）
- 直译：在抗饱和补偿中，先设计一个不显式考虑饱和约束的线性控制器（通常用标准线性设计工具）；在该控制器设计完成后，再设计所谓的抗饱和补偿器来处理饱和约束。
- **可用性**：这是“AW 是事后补充、不改标称控制器”的原始表述，正好支撑笔记里“AW 只在饱和时接管”的判断。

### Q06-2 LMI 综合

> “Many LMI-based approaches now exist to adjust the anti-windup gains in a systematic way … Most often, these are based on the optimisation of either a stability domain or a non-linear L2-induced performance level.”

- 出处：同上，综述正文
- 直译：目前已存在许多基于 LMI 的方法来系统地调节抗饱和增益……它们大多基于对稳定域或非线性 L2 诱导性能水平的优化。

### Q06-3 真实装置：dual-loop VSPID（**笔记需要改**）

> “In this paper, we propose a dual-loop VSPID controller with anti-windup to reduce the integral-windup effect. We tested the proposed method using an real autonomous underwater vehicle and conÞrmed that the method successfully decreased the integral-windup effect.”
>
> “the proposed controller signiÞcantly reduced the overshoot as well as the settling time.”

- 出处：`L06-2_kim2013variable.pdf` 摘要 + 结论
- 直译：本文提出一种带抗饱和的**双环 VSPID** 控制器以减小积分饱和效应；我们在**真实 AUV** 上测试了该方法，确认其成功降低了积分饱和效应。……该控制器显著降低了超调与调节时间。
- **与笔记的冲突（重要）**：
  - 笔记写“按误差幅值在 PD 与 PID 之间切换”——原文是 **dual-loop**（内外环）结构，“变结构”指双环结构，不是按误差切换增益；
  - 笔记把验证平台标为 `△ 待核对`——原文明确是**真艇试验**（Cyclops AUV，surge 与 sway 两个方向）。
- 已按此修正 `01_background_and_model.md` 之外的 L06-2 笔记条目（见 `docs/literature_notes_20261125/02_actuation_and_disturbance.md`）。

---

## Q07 海流

### Q07-1 书里的三条前提 —— ✗ 未取证

同 Q02-2：Fossen 手册的 §8.3 在扫描页内。**目前只有转述，没有原文。** 建议：查纸质书 §8.3.1，或换一个带文本层的版本。

### Q07-2 Al Makdah 2019 —— ✗ 未取证

`L07-2_almakdah2019three.pdf` 的文本层是乱码（自定义字体编码），抽出来像 `/ 0 - n 6 . / 0 n n r r`。文件本身完整（2.5 MB），只是无法机读。需要人工看标题页与摘要。

### Q07-3 高增益观测器估流（含验证方式）

> “In this paper, a high-gain observer (HGO) based on an AUV dynamics model in currents was presented to obtain three-dimensional water current velocities estimates. The current velocities were decided from the difference between the vehicle's absolute velocities and the relative velocities estimated by the model-based HGO. The observer gain for the HGO was determined by solving the Linear Matrix [Inequality] …”

- 出处：`L07-3_kim2018observer.pdf` 摘要
- 直译：本文提出一种基于“含流 AUV 动力学模型”的高增益观测器，用于获得三维水流速度估计；流速由载具绝对速度与模型估计的相对速度之差确定；观测器增益通过求解线性矩阵不等式确定。
- **可用性**：增益用 LMI 求，不是手工调——这修正了笔记里“高增益靠整定、峰化与噪声放大”的暗示（那是对高增益观测器的一般性批评，不是该文的做法）。另据结论段，其估计标准差为 0.0942 m/s，并与 ADCP 对比。

### Q07-4 扰动观测器 + κ 指数稳定

> “The Þrst contribution is an adaptive indirect controller based on a disturbance observer designed for estimation and compensation of ocean currents. The equilibrium points of the cross-track and parameter estimation errors are proven to be globally κ-exponentially stable.”

- 出处：`L13-3_fossen2015direct.pdf` 摘要
- 直译：第一项贡献是基于扰动观测器的自适应间接控制器，用于海流的估计与补偿；横漂误差与参数估计误差的平衡点被证明是全局 κ 指数稳定的。

### Q07-5 只覆盖水平面

> “Both methods can be applied to the horizontal-plane motion of surface vessels and autonomous underwater vehicles (AUVs).”

- 出处：同上，摘要末段
- 直译：两种方法都可应用于水面船舶与自主水下航行器的**水平面运动**。
- **可用性**：这是把“抗流 LOS”从三维缩回水平面的硬证据，本项目的纵向 x–z 平面恰在其外。

---

## 本轮进度

| 主题 | 已取证 | 仍缺 |
|---|---|---|
| L02 | 0 / 4 | Q02-1、Q02-2、Q02-3（书为扫描件）、Q02-4（Antonelli 待查） |
| L03 | 3 / 3 | — |
| L04 | 2 / 2（Q04-1、Q04-2）；Q04-3 已取 | — |
| L05 | 2 / 2 | — |
| L06 | 3 / 3 | — |
| L07 | 3 / 5 | Q07-1（书扫描）、Q07-2（字体乱码） |

**发现的实质性问题两处**：Q04-1 的“二阶可微”是我加的、原文没有；Q06-3 的机制与验证平台都写错了（dual-loop + 真艇，不是误差切换 + 待核对）。两处已在本文件标出，正文笔记同步修正。
