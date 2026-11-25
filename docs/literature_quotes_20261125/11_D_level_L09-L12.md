# D 级取证 · 第二批（L09–L12）

2026-11-25。取证方式同第一批：原版 PDF 文本层 → 逐条定位原句。`[ ]` 表示 OCR 补字，其余保持原样（这批 PDF 的连字 `fi` 常被写成 `Þ`，字母间偶有多余空格）。

---

## Q09 A* 理论

### Q09-1 可采纳性（admissibility）的原始定义

> “We call an algorithm **admissible** if it is guaranteed to find an optimal path from s to a preferred goal node of s for any [finite] graph.”

- 出处：`L09-1_hart1968formal.pdf`，正文（定义段）
- 直译：我们称一个算法是**可采纳的**，若对任意[有限]图，它都保证能找到从 s 到其某个首选目标节点的最优路径。
- **与笔记的差异**：笔记把可采纳性写成对**启发函数** `h ≤ h*` 的约束——这是后人的等价表述；**Hart 1968 原文的可采纳性定义在算法层面**（"保证找到最优路径的算法"）。写论文时若引 Hart，应写"算法可采纳"，若引 `h ≤ h*`，更准确的是引 Dechter & Pearl 或后续教材。

### Q09-1b 加权启发 = 放弃可采纳性（原文自认）

> “Then we could choose an A somewhat larger than the one defined by (3). The algorithm would no longer be admissible, but it might be more desirable … Often, but not always, fewer nodes will be expanded and less arithmetic effort required …”

- 出处：同上，讨论段
- 直译：于是可以取一个比 (3) 式所定义的 A 稍大的值。该算法将不再可采纳，但可能更可取……通常（但并非总是）扩展的节点更少、所需运算量更小。
- **价值**：这是"加权 A*（有界次优换速度）"最早的自认，可替代 Q09-3 用来支撑"放松可采纳性可以更快"。

### Q09-2 / Q09-3 —— ✗ 未取证

`L09-2_dechter1985generalized.pdf` 是**扫描件**，文本层为空（抽不出任何字符）。Dechter & Pearl 的 "computational optimality" 与 "h ≤ h*" 只能人工翻 PDF 或换版本。

---

## Q10 占据栅格与构型空间

### Q10-1 占据栅格的定义

> “The occupancy grid is a multidimensional random field that maintains stochastic estimates of the occupancy state of the cells in a spatial lattice.”

- 出处：`L10-1_elfes1989using.pdf` 正文首段
- 直译：占据栅格是一个多维随机场，维护空间点阵中各个单元占据状态的随机估计。
- **可用性**：注意原文的"**随机场 / 随机估计**"——栅格本身带不确定性，而多数规划器（含本项目）把它当确定图用，这正是要声明的简化。

### Q10-2 贝叶斯更新在本文中的确切位置

> “This density function is subsequently used in a Bayesian estimation procedure to determine the occupancy grid cell state probabilities.”

- 出处：同上，传感器模型段
- 直译：该密度函数随后被用于一个贝叶斯估计过程，以确定占据栅格单元的状态概率。
- **与笔记的差异**：笔记写成"贝叶斯规则随观测更新、对数几率形式"——**对数几率（log-odds）形式是后来的工程实现**，本文用的是概率形式。引用时不要写成该文引入了 log-odds。

### Q10-3 膨胀 = 构型空间中的 Minkowski 差（用 LaValle 代 Latombe）

> “For any two sets X, Y ⊂ Rⁿ, let their Minkowski difference be defined as X ⊖ Y = {x − y ∈ Rⁿ | x ∈ X and y ∈ Y} … In terms of the Minkowski difference, C_obs = O ⊖ A(0).”

- 出处：`L08-1_lavalle2006planning.pdf` §4.3（C-space obstacle）
- 直译：对任意两个集合 X、Y ⊂ Rⁿ，其 Minkowski 差定义为 X ⊖ Y = {x − y | x ∈ X, y ∈ Y}……用 Minkowski 差表示，构型空间障碍为 C_obs = O ⊖ A(0)。
- **说明**：Latombe 1991 的原书未获取（L10-2 缺失），此处**用 LaValle 作为替代证据**。二者对"膨胀 = Minkowski 运算"的表述一致，但**引用时只能引 LaValle**，不能标成 Latombe。

---

## Q11 捷径化（Geraerts & Overmars 2007）

### Q11-1 Shortcut 的操作定义与"必须无碰"

> “The path pruning technique removes a node i−1 from a path if the local path LP between nodes i and i+2 is collision-free. The most widely applied method is the **Shortcut heuristic** … The Shortcut heuristic takes two random configurations on the path. If the part between these two configurations can be replaced by a new shorter path, produced by a local planner, then the original part is replaced by the new path.”

- 出处：`L11-1_geraerts2007creating.pdf` §3
- 直译：路径剪枝技术在某节点 i 可被删除时将其删除，条件是节点 i 与 i+2 之间的局部路径**无碰**。应用最广的是 **Shortcut 启发式**……它取路径上两个随机位形，若二者之间的部分能被局部规划器产生的更短路径替代，则替换之。
- **可用性**：`collision-free` 是替换的**前置条件**，这正好支撑台账里"每次捷径都要重检碰撞"的说法——不是我的推论，是原文条件。

### Q11-2 高质量路径的两个判据

> “That is, a path should be short and should keep some amount of minimum clearance to the obstacles. Traveling along such a path reduces the chances of collisions due to the difficulty of measuring and controlling the precise position of the robot.”

- 出处：同上，引言
- 直译：也就是说，路径应当短，并应与障碍保持一定的最小间隙。沿这样的路径行驶可降低碰撞几率，因为机器人的精确位置难以测量与控制。
- **与笔记的差异**：笔记的 B 级引文此前只能切到 `... a path should be short and should keep some amount of minimum clearance ...`，现在整句拿到，且后半句给出了**理由**（定位不确定性），可直接用于论证"为什么 clearance 是设计参数"。

---

## Q12 连续曲率与平滑

### Q12-1 CC Steer 的三条性质（结论段原文）

> “CC Steer is the first steering method for a car-like vehicle that computes paths with continuous curvature, upper-bounded curvature and upper-bounded curvature derivative.”

- 出处：`L12-1_fraichard2004reeds.pdf` 结论
- 直译：CC Steer 是首个面向类车载具、能计算出具**连续曲率、曲率上界与曲率导数上界**三条性质的路径的转向方法。
- **可用性**：三条性质并列，正好是"几何可执行性"的完整定义。注意它只谈几何，不含速度/加速度。

### Q12-2 组成材料与"长度不最优"

> “The path computed by CC Steer are made up of line segments, circular arcs and clothoid arcs. They are **not optimal in length**. However, it is shown that they converge toward the optimal ‘Reeds and Shepp’ paths when the curvature derivative upper-bound tends to infinity.”

- 出处：同上，摘要/引言
- 直译：CC Steer 计算的路径由直线段、圆弧与回旋曲线弧构成。它们在长度上**并非最优**。然而可以证明，当曲率导数上界趋于无穷时，它们收敛到最优的 Reeds–Shepp 路径。
- **价值**：这是"平滑 = 用长度换可执行性"的原文依据，而且给出了极限关系（σ→∞ 退回最短路径）。

### Q12-3 解析平滑的真实构造：三次 Bézier（**笔记已改**）

> “The algorithm is based upon **parametric cubic Bézier curves**; thus, it is inherently closed-form in its expression, and the algorithm only requires the maximum curvature to be defined.”
>
> “The curvature of the G1CBS path is discontinuous at the junction points, while the curvature of G2CBS path is continuous over the whole path.”

- 出处：`L12-2_yang2010analytical.pdf` 摘要 + 图 11 讨论
- 直译：该算法基于参数化三次 Bézier 曲线，因此表达式天然是闭式的，且只需给定最大曲率。……G1CBS 路径在接点处曲率不连续，而 G2CBS 路径全程曲率连续。
- **与笔记的冲突（重要）**：笔记原写"线段 + 圆弧的解析拼接、迭代替换折角"，**与原文不符**——原文是 Bézier 曲线构造，不是线弧拼接。且原文自证：**G1 版本曲率不连续**，只有 G2 版本才谈得上曲率连续。笔记已按此改写。
- **顺带修正一个常见误解**："G1 平滑" ≠ "曲率连续"。若本项目声称"平滑后曲率连续"，必须用 G2 版本或 clothoid。

---

## 本轮小结

| 条目 | 状态 |
|---|---|
| Q09-1 | ✅ 拿到（并发现笔记把"算法可采纳"写成了"函数可采纳"） |
| Q09-2 / Q09-3 | ✗ Dechter 1985 是扫描件，无文本层 |
| Q10-1 / Q10-2 | ✅ 拿到；Q10-2 需注意 log-odds 是后人实现 |
| Q10-3 | ✅ 用 LaValle 代 Latombe（须注明替代来源） |
| Q11-1 / Q11-2 | ✅ 拿到，且 Q11-1 证实"无碰"是替换的前置条件 |
| Q12-1 / Q12-2 | ✅ 拿到（clothoid 三性质 + 长度不最优） |
| Q12-3 | ✅ 拿到，**推翻笔记**：Bézier 而非线弧；G1 曲率不连续 |

**第三处实质性问题**：L12-2 的方法描述整段写错（线弧 → Bézier），已在笔记与矩阵中更正。这类错误比"少一条引文"严重得多——如果按原笔记去实现，会做出一个和原文不同的算法，却引着原文的名字。
