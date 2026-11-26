# 03 · 规划、栅格与路径处理（L09–L12）

> **状态更新（2026-11-25）**：本文件写于『只有摘要』的阶段，分级为 A/B/C。现已对 `ref/` 中的原版 PDF 逐条取证，**每条的当前状态见其标题下方**，完整原文与差异说明见 [10](10_D_level_L02-L07.md)–[13](13_D_level_L18-L20.md) 四份 D 级文件。

11 条断言。这一组决定论文里"规划章"能不能说清楚两件事：**A* 到底保证什么**，以及**平滑之后还安不安全**。

---

## L09 A* 理论

### Q09-1 · A* 的形式化目标与"一类策略的最优性"

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：A* 把领域启发信息纳入图搜索的形式化框架，并对一类搜索策略给出最优性性质。
- **来源**：`hart1968formal`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "This paper describes how heuristic information from the domain can be incorporated into a formal mathematical theory of graph searching and demonstrates an optimality property for a class of search strategies."
- **直译**：本文描述了如何把来自领域的启发信息纳入图搜索的形式化数学理论，并论证了一类搜索策略所具有的最优性性质。
- **引用边界**：原文只说"**一类**搜索策略具有最优性性质"，**没有**说"A* 无条件最优"。可采纳性判据在正文，不在摘要——所以这条只能支撑"形式化最优性存在"，不能支撑"我们的 A* 是最优的"。
- **用途**：论文"规划算法"小节的定位句；配合 Q09-3 才能说清最优性的条件。

### Q09-2 · A* 的"计算最优性"是一个专门概念

> **状态**：✗ 未取证（PDF 无文本层/编码异常） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：A* 的计算最优性指它不会扩展任何可被其他算法跳过的节点。
- **来源**：`dechter1985generalized`｜**C 级（已确认无法机读）**
- **原文**：
  > "... the computational optimality of A* ..."
- **直译（释义，非逐字）**：所谓"计算最优性"指 A* 从不扩展一个可被其他拥有同样启发信息的算法跳过的节点。
- **引用边界**：这是**搜索效率**意义上的最优（不浪费扩展），不是路径代价意义上的最优。中文文献常把两者混为一谈，这里必须分开写。该片段在数据库中只能安全切到 "the computational optimality of A*" 四词，后半句（谁跳过谁）**未取得可核原文**，因此只作释义，不作引文。
- **用途**：解释"A* 为什么在同等启发下扩展节点最少"。

### Q09-3 · 最优性的条件是"乐观估计"

> **状态**：✗ 未取证（PDF 无文本层/编码异常） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：当所有代价估计都是乐观的（即 `h ≤ h*`）时，该类算法返回最优解。
- **来源**：`dechter1985generalized`｜**C 级（已确认无法机读）**
- **原文**：
  > "... when all cost estimates are optimistic (i.e., h ≤ h*) ..."
- **直译**：……当所有代价估计都是乐观的（即 h ≤ h*）时……
- **引用边界**：`h ≤ h*` 是**可采纳性**的等价表述。原文这一片段在摘要中被截断，只保留这半句，不可补写"则 A* 最优"以外的内容（例如不要引申为"启发越强越好"）。
- **待办**：正文里找 "admissible" 的正式定义段，升 D 级；这是论文"启发函数选择"一节的立论句。

---

## L10 占据栅格与膨胀

### Q10-1 · 占据栅格是什么（含"随机场"表述）

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：占据栅格用概率化的离散表示描述空间占据状态，并以随机场形式维护每个栅格的占据估计。
- **来源**：`elfes1989using`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "An approach to robot perception and world modeling that uses a probabilistic tesselated representation of spatial information called the occupancy grid is reviewed."
  >
  > "The occupancy grid is a multidimensional random field that maintains stochastic estimates of the occupancy state of the cells in a spatial lattice."
- **直译**：本文综述一种用于机器人感知与世界建模的方法，它使用称为占据栅格的概率化镶嵌式空间信息表示。占据栅格是一个多维随机场，维护空间点阵中各单元占据状态的随机估计。
- **引用边界**：原文强调"**估计**"与"随机场"——即栅格天然带不确定性。而多数规划器（包括本项目）把栅格当**确定图**用，这是必须声明的简化。
- **用途**：论文"地图表示"一节的定义句；也是"为什么本项目的碰撞判定是确定性的"这一局限的来源。

### Q10-2 · 贝叶斯增量更新与多视角融合

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：栅格由多传感器、多视角读数经贝叶斯估计增量更新。
- **来源**：`elfes1989using`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "Bayesian estimation procedures allow the incremental updating of the occupancy grid, using readings taken from several sensors over multiple points of view."
- **直译**：贝叶斯估计程序允许对占据栅格做增量更新，所用读数来自多个传感器、多个视角。
- **引用边界**：原文的"贝叶斯"针对**感知融合**。若本项目的栅格是直接由场景几何生成的（无感知），则**不适用**这句——引用它反而会让人误以为我们做了感知融合。
- **用途**：仅当论文讨论"若接入真实声呐该怎么做"时引用；不要用在方法章描述本项目地图生成。

### Q10-3 · 障碍膨胀是构型空间操作

> **状态**：✅ 已取证 D 级（用 LaValle §4.3 代 Latombe） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：障碍膨胀等价于在构型空间中把障碍按载具外形扩张，从而使"点机器人"的避障等价于原问题。
- **来源**：`latombe1991robot`（专著）｜**D 级（LaValle 代 Latombe）**
- **定位**：构型空间一节，障碍的 Minkowski 和定义（`C_{obs} = O ⊕ (−R)` 形式）与"clearance"讨论。
- **检索锚点**：`configuration space`、`Minkowski`、`C-obstacle`、`clearance`
- **引用边界**：等价性只在**精确**构型空间成立；栅格近似下退化为分辨率意义下的近似。这句话是本项目"安全距离不是物理保证"的唯一硬依据，务必取到原文。
- **待办**：无（已取证，见 D 级文件）

---

## L11 捷径化

### Q11-1 · Partial shortcut：以插值方式去除冗余运动

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：捷径化通过用无碰直线段替换子路径来缩短路径，每次替换都必须重新做碰撞检查。
- **来源**：`geraerts2007creating`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "This paper reports a new technique, called Partial shortcut, which decreases the path length."
- **直译**：本文报告一种称为部分捷径化的新技术，它可减小路径长度。
- **引用边界**：原文只说"减小长度"，**没有**承诺保持间隙。把"每次替换都重新碰撞检查"写成该文结论前，必须在正文找到对应表述（通常出现在算法伪代码或"collision check"段落）。
- **待办**：无（已取证，见 D 级文件）

### Q11-2 · "高质量路径"的两个判据

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：高质量路径既要求短，也要求保留一定的最小间隙。
- **来源**：`geraerts2007creating`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "... a path should be short and should keep some amount of minimum clearance ..."
- **直译**：……路径应当短，并应保持一定量的最小间隙……
- **引用边界**：`some amount` 是模糊量——该文没有给"多少算够"。本项目若设 0.5 m 之类的阈值，属于自定参数，不可引该文背书。
- **价值**：这句话正是"捷径化会让路径贴障碍"的**问题陈述**，可以直接用来论证"我们为什么要做平滑后 clearance 重检"。
- **用途**：论文"路径后处理"一节的动机句。

---

## L12 平滑与曲率连续

### Q12-1 · 连续曲率路径的三条性质

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：连续曲率（CC）路径同时满足曲率连续、曲率有上界、曲率导数有上界三条性质。
- **来源**：`fraichard2004reeds`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "CC Steer is the first to compute paths with: 1) continuous curvature; 2) upper-bounded curvature and 3) upper-bounded curvature derivative."
- **直译**：CC Steer 是首个能计算具备以下性质的路径的方法：1）曲率连续；2）曲率有上界；3）曲率导数有上界。
- **引用边界**：这三条是**几何可执行性**，不是动力学可执行性（不含速度、加速度、侧滑）。用它说明"曲率不连续 ⇒ 舵角阶跃 ⇒ 不可执行"是恰当的；用它说明"轨迹一定可跟踪"则越界。
- **用途**：论文"平滑"一节的核心定义；反驳"圆弧倒角就够了"的理论依据。

### Q12-2 · CC 路径的构造材料，以及它不最优

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：CC 路径由直线段、圆弧与回旋曲线拼接而成；代价是路径长度不再最优。
- **来源**：`fraichard2004reeds`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "The paths computed by CC Steer are made up of line segments, circular arcs, and clothoid arcs."
  >
  > "They are not optimal in length."
- **直译**：CC Steer 计算出的路径由直线段、圆弧与回旋曲线弧构成。它们在长度上并非最优。
- **引用边界**：第二句是本条最重要的部分——**连续曲率的代价就是长度**。论文里做平滑时必须承认这一点，否则"平滑不损失最优性"是假话。
- **用途**：论文"平滑的代价"讨论；矩阵"路径平滑"列的注解。

### Q12-3 · 解析平滑与三维平滑（两条 C 级）

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[11_D_level_L09-L12.md](11_D_level_L09-L12.md)

- **拟写入论文**：解析式线—弧平滑以闭式构造保证切向连续与曲率有界，计算量小；三维平滑则需额外处理俯仰与挠率约束。
- **来源**：`yang2010analytical`（解析平滑）｜**D 级（原文，见 D 级文件）**；`park2022three`（三维 Dubins 引导平滑）｜**C 级（未单独取证）**
- **定位**：Yang 2010 摘要 + 算法节（曲率界、切向连续）；Park 2022 摘要 + 三维构造节。
- **检索锚点**：Yang：`continuous curvature`、`line and arc`、`curvature bound`、`G1`；Park：`Dubins`、`3D`、`torsion`、`pitch`
- **引用边界**：Yang 的"曲率连续"通常不等于"曲率导数连续"（那是 clothoid 的性质）。两者混用是常见审稿意见。
- **待办**：无（已取证，见 D 级文件）

---

## 本组提醒

- Q09-1 与 Q09-3 必须**成对出现**：只引前者会显得 A* 无条件最优，只引后者会丢掉"形式化框架"这一历史贡献。
- Q12-2 的"They are not optimal in length"是本台账里最有防守价值的一句：它把"平滑"从"改进"降格为"交换"，而这正是本项目要说的话。
