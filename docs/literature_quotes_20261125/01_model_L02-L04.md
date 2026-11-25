# 01 · 模型、降维与欠驱动（L02–L04）

10 条断言。这一组的每一条都会直接进"模型章"，是最容易被审稿人追问"你凭什么这么建模"的地方，因此限定词要抄足。

---

## L02 海洋载具 6-DOF 动力学

### Q02-1 · 6-DOF 矢量模型的标准形式

- **拟写入论文**：载具动力学采用 6-DOF 矢量形式 `M ν̇ + C(ν)ν + D(ν)ν + g(η) = τ`，其中惯性矩阵含刚体与附加质量两部分。
- **来源**：`fossen2011handbook`（Wiley 专著）｜**C 级**
- **定位**：动力学部分"刚体动力学 / 水动力与阻尼"两节，矢量方程首次出现处；惯量分解为 `M = M_RB + M_A`。
- **检索锚点**：`M_{RB}`、`M_A`、`C(\nu)\nu`、`D(\nu)\nu`、`g(\eta)`、`\tau`
- **为什么必须是 C**：专著无摘要，无法取得逐字句；且不同版次章节号可能变动。
- **引用边界**：这是通用海洋载具方程，**不是**某型 AUV 的辨识结果。论文中须写成"采用 Fossen 的通用形式"，不可写成"该模型已被验证适用于本艇"。
- **待办**：定位到该式，抄下式号（如 `(3.16)`）与前后各一句说明，升为 D 级。

### Q02-2 · 海流的唯一合法入口是相对速度

- **拟写入论文**：海流不作为附加力项，而是通过相对速度 `ν_r = ν − ν_c` 进入阻尼与科氏项。
- **来源**：`fossen2011handbook`｜**C 级**
- **定位**：环境载荷/海流小节，`\nu_r \triangleq \nu - \nu_c` 的定义式及其后的三条前提（irrotational、constant、inertial-frame）。
- **检索锚点**：`\nu_r`、`\nu_c`、`irrotational`、`constant current`
- **引用边界**：这一形式自带"恒定、无旋、空间均匀"三个假设。写"我们考虑海流"时必须同时写这三条，否则就是隐含地宣称能处理时变流。
- **待办**：抄下三条假设的原文句——这是本台账里**最值得花时间**的一条，因为全项目的"恒流"合法性都挂在它上面。

### Q02-3 · 附加质量取常数矩阵、忽略频率依赖

- **拟写入论文**：模型采用常数附加质量矩阵，忽略辐射记忆效应与频率依赖。
- **来源**：`fossen2011handbook`｜**C 级**
- **定位**：附加质量小节开头的建模假设段（通常伴随"zero-frequency"或"frequency-independent"字样）。
- **检索锚点**：`added mass`、`frequency`、`memory effect`、`zero-frequency`
- **引用边界**：这是**简化**而非事实。论文里应写成局限："常数附加质量在快速机动下会低估瞬态水动力"。
- **待办**：找到承认该简化的原句，升 D 级；若找不到（很多教材默认处理），则在正文注明"该简化由本文引入"。

### Q02-4 · 欠驱动的代数判据

- **拟写入论文**：欠驱动的严格表述是推进器配置矩阵的秩小于自由度维数，即 `rank(B) < 6`。
- **来源**：`antonelli2018underwater`（Springer 专著）｜**C 级**
- **定位**：推力配置矩阵小节，`\tau = B u` 的定义与秩/可控子空间讨论。
- **检索锚点**：`B`、`thruster configuration matrix`、`rank`、`controllable`
- **引用边界**：秩判据说明"某些方向不可达"，但**不**说明"不可达程度"；可达集大小要另算（见 Q05-1）。
- **待办**：抄 `\tau = B u` 定义句 + 关于秩的一句，升 D 级。

---

## L03 纵向平面降维

### Q03-1 · 6-DOF 模型 + 分通道滑模自动驾驶仪

- **拟写入论文**：Healey 与 Lienard 在完整 6-DOF 模型上设计了覆盖操舵、下潜与速度的滑模自动驾驶仪。
- **来源**：`healey1993multivariable`｜**A 级（摘要）**
- **原文**：
  > "A six-degree-of-freedom model for the maneuvering of an underwater vehicle is used and a sliding-mode autopilot is designed for combined steering, diving, and speed control functions."
- **直译**：采用一个用于水下航行器机动的六自由度模型，并设计了一个滑模自动驾驶仪，用于联合的操舵、下潜与速度控制功能。
- **引用边界**：原文只说"联合的 steering / diving / speed"，**没有**说"三个通道完全独立"。把"分通道设计"写成它的原话属于越界；正确表述是"以联合方式设计了覆盖三类功能的自动驾驶仪"。
- **用途**：L03-1 笔记的"贡献"栏；模型章说明"6-DOF → 功能分解"的历史依据。

### Q03-2 · 解耦建模是被显式声明的假设

- **拟写入论文**：该设计的有效性建立在一个显式声明的解耦建模假设之上。
- **来源**：`healey1993multivariable`｜**A 级（摘要片段，用省略号）**
- **原文**：
  > "a multivariable sliding-mode autopilot based on state feedback, designed assuming decoupled modeling, ..."
- **直译**：一种基于状态反馈、**在设计时假定解耦建模**的多变量滑模自动驾驶仪……
- **引用边界**：`assuming decoupled modeling` 是作者自认的假设，这正是本项目"降维合法性"论证要引的那句话——但它是 1993 年在特定艇型与工况下成立的假设，不能升级为普遍结论。
- **价值**：这是整份台账里**唯一**能逐字证明"解耦是假设而非事实"的句子，务必用上。
- **用途**：论文"模型简化假设"一节的直接引文。

### Q03-3 · LOS 在该文中的原始角色

- **拟写入论文**：LOS 引导最初是作为航路点捕获（waypoint acquisition）手段与自动驾驶仪配合使用的。
- **来源**：`healey1993multivariable`｜**A 级（摘要片段）**
- **原文**：
  > "... line-of-sight guidance is used to achieve path tracking."
- **直译**：……使用视线（LOS）引导来实现路径跟踪。
- **引用边界**：原文只说"用于实现路径跟踪"，没给前视距离、没给稳定性条件。**不要**用它支撑"LOS 参数如何选取"。
- **用途**：L13-1 的历史铺垫；说明"LOS 早于其形式化分析"。

---

## L04 欠驱动与可行性

### Q04-1 · 参考轨迹不需要特定类型，但必须足够光滑有界

- **拟写入论文**：对欠驱动载具，可跟踪的参考轨迹不必是特定类型（如配平轨迹），但必须是一条足够光滑、有界、以时间为参数的曲线。
- **来源**：`aguiar2007trajectory`｜**A 级（摘要片段，两段）**
- **原文**：
  > "The desired trajectory does not need to be of a particular type (e.g., trimming trajectories) ... any sufficiently smooth bounded curve parameterized by time."
- **直译**：期望轨迹不需要属于某一特定类型（例如配平轨迹）……任何足够光滑、有界、以时间为参数的曲线即可。
- **引用边界**：这是"可行性条件"的**弱形式**（光滑 + 有界），不是"满足执行器约束"。用它论证"几何折线需要先平滑"是恰当的；用它论证"该路径一定可执行"则是过度引申。
- **⚠ 需回改笔记**：L04-1 笔记里写的"3H △（水平面三自由度）"不准确——摘要原话是"either 2- or 3-D space"的一般类，案例为 2-D 气垫船与 3-D 水下航行器。已在下一次修订中改正。

### Q04-2 · 自适应切换监督 + Lyapunov 跟踪律

- **拟写入论文**：该文把自适应切换监督控制与基于 Lyapunov 的非线性跟踪律结合，使位置跟踪误差收敛到原点附近任意小的邻域。
- **来源**：`aguiar2007trajectory`｜**A 级（摘要整句）**
- **原文**：
  > "For a general class of moving vehicles in either 2- or 3-D space, we demonstrate how adaptive switching supervisory control can be combined with a nonlinear Lyapunov-based tracking control law to solve the problem of global boundedness and convergence of the position tracking error to a neighborhood of the origin that can be made arbitrarily small."
- **直译**：对 2 维或 3 维空间中一般类别的运动载具，我们演示了如何将自适应切换监督控制与非线性 Lyapunov 跟踪控制律相结合，以解决位置跟踪误差的全局有界性，并使其收敛到原点附近一个可任意小的邻域。
- **引用边界**：结论是"误差到任意小邻域"，**不是**"误差到零"，也不是"满足输入约束"。"切换监督"意味着控制器族切换，与单一 PID 结构不同——引它时不能说"PID 也能做到"。
- **用途**：L04-1 贡献句；"为什么参数不确定下仍需模型"的论据。

### Q04-3 · 深度通道只能经纵倾间接控制

- **拟写入论文**：欠驱动 AUV 无垂向推力，深度偏差只能通过纵倾指令间接消除。
- **来源**：`gao2016depth`｜**C 级**
- **定位**：摘要首段（"depth control ... vertical plane ... optimal internal model controller"）与正文深度—纵倾两级结构处。
- **检索锚点**：`internal model`、`depth`、`pitch`、`vertical plane`、`underactuated`
- **引用边界**：这是**功能结构**陈述，不是性能结论。该文的"最优"指内模控制器的参数整定，不等于全局最优。
- **待办**：抄摘要前两句 + 深度—纵倾传递关系的一句，升 D 级。

---

## 本组提醒

这一组里有 6 条是 C 级（全部来自三部专著与一篇会议论文）。原因是：**教材与专著的"假设"往往写在正文而不在摘要**，而本项目最需要的恰恰是那些假设。建议优先补 Q02-2、Q02-3、Q02-4 三条——它们同时支撑"模型章"和"局限章"。
