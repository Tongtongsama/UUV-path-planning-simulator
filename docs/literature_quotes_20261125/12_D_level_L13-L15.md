# D 级取证 · 第三批（L13–L15）

2026-11-25。`[ ]` 为 OCR 补字。这一批的两个源文件文本层质量差：Fossen 2003（L13-1）断词严重，且 `κ`、`ψ` 等希腊字母被整段丢弃；因此凡引文含缺字符，一律用 `[ ]` 标出并在"可用性"里说明。

---

## Q13 LOS 引导

### Q13-1 Fossen, Breivik & Skjetne 2003——LOS 的两个组成部分

> “First, a LOS guidance procedure is derived. This includes a projection algorithm and a way-point switch[ing] algorithm. To avoid large bumps in [ψ]d when switching, and to provide the necessary derivatives of [ψ]d to the controller, the commanded LOS heading is fed through a reference model.”

- 出处：`L13-1_fossen2003line.pdf` §1（引言末）
- 直译：首先推导 LOS 引导过程，它包含一个投影算法与一个航路点切换算法。为避免切换时期望航向 ψd 出现大幅跳变，并为控制器提供 ψd 的必要导数，期望 LOS 航向经过一个参考模型。
- **与笔记的差异**：笔记只说"前视距离 + 横向误差 → 期望航向"，漏掉了原文的两件工程必需件——**投影算法**（把载具投影到路径上）与**参考模型**（平滑切换、提供导数）。这两件恰恰是"LOS 能不能直接接 PID"的关键：没有参考模型，航路点切换处的航向阶跃会让内环饱和。
- **可用性**：该文件 OCR 质量差（`ψd` 被丢、单词粘连），引用时建议只引前半句，或直接引原文 PDF 页码。

### Q13-2 积分 LOS 的适用范围

> 摘要（OpenAlex 摘要，PDF 文本层此处丢字）：“… a new κ-exponentially stable integral LOS guidance law, capable of eliminating the effect of constant external disturbances in **straight-line path following**, is derived.”

- 出处：`L13-2_lekkas2014integral.pdf` 摘要（PDF 文本层把 "constant environmental forces in straight-line path following" 整段丢了，只能用摘要重建）
- 直译：……推导出一种新的 κ 指数稳定积分 LOS 引导律，能够消除**直线路径跟踪**中常值外扰的影响。
- **补充证据（正文 §VII）**：“In this section, we will present a modiÞed version of the integral LOS algorithm based on the system kinematics.”——积分律在正文里确实是对直线情形给出的改进版。
- **结论**：Q13-2 的"限直线"判断成立，笔记已按此改写。

### Q13-3 时变前视距离

> “Regarding the LOS guidance problem, we presented a simple extension of the conventional approach by proposing a **time-varying lookahead distance**, which depends on the cross-track error and a few tuning constants. This contributed to obtaining a more flexible approach, since different lookahead distances result in different maneuvering behaviors.”

- 出处：结论段
- 直译：关于 LOS 引导问题，我们通过提出**时变前视距离**对传统方法做了简单扩展，该距离依赖横漂误差与若干整定常数。这带来了更灵活的方法，因为不同前视距离会产生不同的操纵行为。
- **可用性**：这句把"Δ 是设计自由度"说透了——Δ 不是唯一的，选不同的 Δ 就是选不同的操纵风格。正好支撑本项目"Δ 敏感性实验"的必要性。

### Q13-4 级联结构与 κ 指数稳定

> “The guidance system along with a heading controller form a cascaded structure, which is shown to be [κ]-exponentially stable when the control task is to converge to the path produced by the aforementioned CHSI method.”

- 出处：摘要
- 直译：引导系统与航向控制器构成级联结构，在控制任务是收敛到前述 CHSI 方法所生成路径的情形下，被证明是 κ 指数稳定的。
- **注意**：κ 在 PDF 文本层被丢弃（显示为 "-exponentially"）。这正说明**不能只靠文本层**：符号级内容必须看 PDF 页面。

### Q13-5 依赖相对速度测量

> “The control laws are implemented using **hydro-acoustic relative velocity measurements** as opposed to absolute velocity measurements. For this purpose a kinematic model for relative velocity in amplitude-phase form is derived.”

- 出处：`L13-3_fossen2015direct.pdf` 摘要
- 直译：这些控制律用**水声相对速度测量**而非绝对速度测量来实现。为此推导了幅值—相位形式的相对速度运动学模型。
- **对本项目**：本项目用仿真真值速度，等于免费拿到了这一测量，必须在局限里写明。

---

## Q14 PID 类

### Q14-1 —— ⚠ 用替代文献支撑（原 L14-1 为 OMAE 会议论文，未获取）

结论层面的支撑存在，但**不能声称"分通道 PID 是该文做法"**：

> “A novel control architecture based on model-free control principles is presented to guarantee stable and precise trajectory tracking performance … in the proposed hybrid controller, intelligent-PID (i-PID) and PD feedforward controllers are combined …”

- 出处：`L14-2_bingul2023intelligent.pdf` 摘要
- **处理**：台账 Q14-1 的级别保持 C（没有原文），只在"相关资料"里指明可用 L14-2 + `_substitutes/` 的 SUB-L14-2 阅读，不得作为 Q14-1 的证据。

### Q14-2 智能 PID 的超局部模型

> “… in the model-free approach, this mathematical model is represented by a structure called the **ultra-local model**, which is ẏ⁽ʰ⁾ = F + αu … Parameter h is the derivation order which is greater than or equal to 1, and α is the non-physical constant parameter. F is the quantity representing the real dynamics of the AUV as well as the external disturbances for each axis of six DOF.”

- 出处：`L14-2_bingul2023intelligent.pdf` §3（式 (25) 前后）
- 直译：在无模型方法中，该数学模型被表示为称为**超局部模型**的结构：y⁽ʰ⁾ = F + αu。h 为不小于 1 的求导阶数，α 是非物理常数参数，F 表示 AUV 的真实动态与外部扰动（对六自由度的每个轴分别定义）。
- **可用性**：`α 是非物理常数`是原文自认——这正好支撑笔记里"iPID 不是无参数，只是参数没有物理意义"的判断。

---

## Q15 规划—控制耦合

### Q15-1 Donald et al. 1993 —— ✗ 未取证

`L15-1_donald1993kinodynamic.pdf` 是**扫描件**，文本层为空（抽取 0 字符）。kinodynamic planning 的定义只能引 LaValle（`L08-1` 第 13 章"Velocity constraints on the configuration space"）或人工翻 PDF。

### Q15-2 Khorrami & Krishnamurthy 2009——分层的具体分工

> “… the proposed MUNS has a **hierarchical architecture** comprising of a **Wide Area Planner (WAP)** and a **Local Area Planner (LAP)**. … The WAP is responsible for higher-level planning given the mission speciÞcations while the LAP focuses on local obstacle avoidance. The LAP can operate based on just the relative measurements to obstacles provided by the sonars at the current time …”

- 出处：`L15-2_khorrami2009hierarchical.pdf` 摘要 + §II
- 直译：所提出的 MUNS 采用**分层架构**，由**广域规划器（WAP）**与**局部规划器（LAP）**组成。WAP 依据任务要求做高层规划，LAP 专注于局部避障；LAP 可以仅依赖声呐当前提供的相对测量运行。
- **可用性**：这是"高层不知道低层动力学极限"的典型架构证据——WAP 按任务规划、LAP 只用声呐相对测量，两者之间**没有**转弯半径/减速距离这类可行性契约。写"分层架构存在可行性缺口"时应引它。
- **与笔记的差异**：笔记说"两层通过航路点序列与速度指令衔接"，原文更具体：WAP 输出全局航迹建议（Global Trajectory Recommendation），LAP 输出线速度与角速度指令（Linear/angular velocity commands）——见文末框图文字。

---

## 本轮小结

| 条目 | 状态 |
|---|---|
| Q13-1 | ✅ 拿到，且补出笔记漏掉的**投影算法 + 参考模型** |
| Q13-2 | ✅ 成立（摘要重建 + 正文 §VII 佐证） |
| Q13-3 | ✅ 拿到，Δ 作为"设计自由度"的原文依据 |
| Q13-4 | ✅ 拿到（κ 符号在文本层丢失） |
| Q13-5 | ✅ 拿到 |
| Q14-1 | ✗ 无原文；替代文献不能充当该条证据 |
| Q14-2 | ✅ 拿到（超局部模型式 (25)，α 非物理常数） |
| Q15-1 | ✗ Donald 1993 扫描件，无文本层 |
| Q15-2 | ✅ 拿到，且比笔记更具体（WAP→航迹建议，LAP→速度指令） |

**新发现一处笔记遗漏**：L13-1 的 LOS 结构里还有"参考模型"一环，它负责平滑航路点切换并给控制器提供 ψd 的导数。缺了这一环，"LOS 直接接 PID"在航路点切换处会出现航向阶跃——这正是本项目要处理的实际问题。
