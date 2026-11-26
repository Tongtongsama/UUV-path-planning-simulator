# D 级取证 · 第四批（L02-4、L18–L20）

2026-11-25。这一批里有两个重要发现：**LOTUSim 自带分层海流模型**（与本项目的恒流设定直接相关），以及 **rosbag 并不在 ROS 2009 原文里**。

---

## Q02-4 推进器配置矩阵的秩

### 原文依据：TCM 与"全驱动/冗余"的判据

> “TCM — Thruster Control Matrix”
>
> “… the thruster conﬁguration for an underwater vehicle with 8 thrusters … The vehicle is both fully actuated and redundant. Finally, an example of fully actuated vehicle with the minimum number of thrusters is given in Fig. 2.21.”

- 出处：`L02-2_antonelli2018underwater.pdf` 符号表 + §2 推进器配置节
- 直译：TCM 即推进器控制矩阵。……某 8 推进器水下航行器的配置……该载具既全驱动又冗余。最后，图 2.21 给出了"推进器数量最少"的全驱动载具示例。
- **与笔记的差异（重要）**：笔记写"欠驱动的严格表述是 `rank(B) < 6`"。**Antonelli 原文并没有用秩来定义欠驱动**，它用的是"全驱动 / 冗余 / 某方向不可驱动"这套配置语言。`rank(B) < 6` 是我做的代数化改写——数学上没错，但**不能标成 Antonelli 的定义**。
- **建议写法**："推进器配置矩阵 `τ = B u` 的行秩决定可控子空间（本文据此把欠驱动表述为 `rank(B) < 6`；Antonelli 用'全驱动/冗余/不可驱动'描述同一事实）"。

---

## Q18 仿真平台

### Q18-1 / Q18-2 Gazebo 的动机与定位

> “To date, most simulators have been restricted to 2D worlds, and few have matured to the point where they are both highly capable and easily adaptable. Gazebo is designed to fill this niche by creating a 3D dynamic multi-robot environment capable of recreating the complex worlds that will be encountered by the next generation of mobile robots.”

- 出处：`L18-1_koenig2004design.pdf` 摘要（OCR：`slrategies`、`effident`）
- 直译：迄今为止多数仿真器局限于 2D 世界，鲜有成熟到既功能强大又易于适配者。Gazebo 旨在填补这一空白，通过构建 3D 动态多机器人环境来复现下一代移动机器人将面对的复杂世界。
- **注意**：这句话**完全没有涉及流体/浮力/海流**。把它当作"Gazebo 支持水动力"的依据是错的。

### Q18-3 LOTUSim：自带分层海流模型（**与本项目直接相关**）

> “A high-ﬁdelity underwater current model for real-time simulation, with an **Ekman-inspired layered current model**, validated against oce[an data] …”
>
> “LOTUSim is a multi-domain maritime simulator that jointly models surface, underwater, and aerial dynamics … It leverages Gazebo's wind modeling and introduces custom surface and underwater models, including a layered ocean current model inspired by Ekman …”

- 出处：`L18-2_buche2026lotusim.pdf` 贡献列表 + §环境建模
- 直译：面向实时仿真的高保真水下海流模型，采用受 Ekman 启发的**分层海流模型**，并已用海洋数据验证……LOTUSim 联合建模水面、水下与空中动态，复用 Gazebo 的风场建模并引入自定的水面/水下模型，含 Ekman 启发的分层海流模型。
- **对本项目的影响（重要）**：本项目用的是**常值流**，而平台本身支持**分层流**。论文里应明确写"我们只启用了恒流档，平台的分层流能力未被使用"，否则会被问"为什么不用平台已有的分层流"。
- **与笔记的差异**：笔记把 LOTUSim 的流体能力写成"可配置流体 △"，现在可以确认它**确有分层流模型且做过验证**。

### Q18-4 ROS 的原始卖点（以及 rosbag 不在其中）

> “In contrast, peer-to-peer connectivity, combined with buffering or fanout software modules where necessary, avoids the issue entirely. … To support cross-language development, ROS uses a simple, language-neutral interface deﬁnition language (IDL) to describe the messages sent between modules.”

- 出处：`L18-3_quigley2009ros.pdf` §II
- 直译：相比之下，点对点连通性（必要时配合缓冲或扇出模块）完全避免了该问题。……为支持跨语言开发，ROS 使用一种简单的、语言中立的接口定义语言（IDL）来描述模块间发送的消息。
- **与笔记的冲突**：笔记把"录包 → 离线重放"当作该文的贡献。**`rosbag` 在 2009 年的这篇 workshop 论文里没有出现**，它是后来工具链的产物。引用可复现性时，要么引 ROS 后来的文档，要么把"记录—重放"写成"ROS 生态提供的能力"而不是"该文提出"。

---

## Q19 验证与确认

### Q19-1 Sargent：四个动作的确切名称

> “Conceptual Model Validation … Specification Veriﬁcation … Implementation Veriﬁcation … Operational (Results) Validation”

- 出处：`L19-1_sargent2013verification.pdf` 图 3 及其说明
- 直译：概念模型确认、规格验证、实现验证、运行（结果）确认。
- **与笔记的差异**：笔记只写"验证=模型建得对不对、确认=建的是不是对的模型"两句。原文是**四个**环节，而且把"概念模型确认"（问题实体→概念模型）与"运行确认"（仿真结果→真实系统）分开了——这比二分法细。
- **可用性**：写论文的验证章时，"我们做了坐标/时间一致性检查"属于 **implementation verification**；"我们与解析解对比"属于 **operational validation** 的弱形式。用原文术语比自造词稳。

### Q19-2 Oberkampf —— ⚠ 用替代文献（Thacker 2004）

> “Model veriﬁcation and validation (V&V) is an enabling methodology for the development of computational models that can be used to make engineering predictions with quantiﬁed conﬁdence.”

- 出处：`_substitutes/SUB-L19-1_benhthacker2004.pdf` 摘要
- 直译：模型验证与确认（V&V）是一套使能方法学，用于开发能够以可量化的置信度做出工程预测的计算模型。
- **说明**：Oberkampf & Roy 的书未获取，此处是**替代证据**。它支撑"V&V 的目的是带量化置信度的预测"这一层，但**支撑不了**该书特有的"代码验证/解的验证/验证层级"三分法——那部分只能引原文或用 Thacker 的相近章节替代。

---

## Q20 可复现

### Q20-1 Sandve 的十条规则（标题原文）

> “Rule 1: For Every Result, Keep Track of How It Was Produced”
> “Rule 2: Avoid Manual Data Manipulation Steps”
> “Rule 3: Archive the Exact Versions of All External Programs Used”
> “Rule 4: Version Control All Custom Scripts”
> “Rule 6: For Analyses That Include Randomness, Note Underlying Random Seeds”
> “Rule 10: Provide Public Access to Scripts, Runs, and Results”

- 出处：`L20-1_sandve2013ten.pdf` 各节标题
- 直译：规则 1：对每个结果记录它是如何产生的；规则 2：避免手工的数据操作步骤；规则 3：归档所用外部程序的精确版本；规则 4：对所有自写脚本做版本控制；规则 6：含随机性的分析要记录所用随机种子；规则 10：公开脚本、运行与结果。
- **可用性**：这几条正好对应本项目 C6 的证据链（配置冻结、脚本化、依赖版本、随机种子）。引用时按规则编号引，比笼统说"遵循可复现原则"有力得多。

---

## 本轮小结

| 条目 | 状态 | 备注 |
|---|---|---|
| Q02-4 | ✅ 拿到，但**秩判据是我的改写** | 写作时需区分原文措辞与代数化表述 |
| Q18-1 / Q18-2 | ✅ | 原文与流体无关，勿引申 |
| Q18-3 | ✅ **并推翻笔记的含糊说法** | LOTUSim 有 Ekman 分层流模型且做过验证 |
| Q18-4 | ✅ **并推翻笔记** | rosbag 不在 2009 原文中 |
| Q19-1 | ✅ 且比笔记细 | 四个环节而非二分 |
| Q19-2 | ⚠ 替代证据 | 只能支撑"V&V 目的"这一层 |
| Q20-1 | ✅ | 十条规则标题可直接引用 |

**两处笔记需改**：L18-3（分层流能力）、L18-4（rosbag 归属）。已在下一条消息同步修改。
