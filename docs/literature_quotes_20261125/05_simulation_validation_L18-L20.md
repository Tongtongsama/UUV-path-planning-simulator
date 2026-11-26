# 05 · 仿真平台、V&V 与可复现（L18–L20）

> **状态更新（2026-11-25）**：本文件写于『只有摘要』的阶段，分级为 A/B/C。现已对 `ref/` 中的原版 PDF 逐条取证，**每条的当前状态见其标题下方**，完整原文与差异说明见 [10](10_D_level_L02-L07.md)–[13](13_D_level_L18-L20.md) 四份 D 级文件。

7 条断言。这一组不提供方法，提供的是**论文可信度的边界**。写得好，审稿人挑不动；写得虚，一句"你这也叫验证？"就够了。

---

## L18 仿真平台

### Q18-1 · Gazebo 出现之前，多数仿真器被困在 2D

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[13_D_level_L18-L20.md](13_D_level_L18-L20.md)

- **拟写入论文**：在 Gazebo 之前，多数机器人仿真器局限于二维世界，且很少同时做到"能力强"与"易适配"。
- **来源**：`koenig2004design`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "To date, most simulators have been restricted to 2D worlds, and few have matured to the point where they are both highly capable and easily adaptable."
- **直译**：迄今为止，多数仿真器局限于二维世界，鲜有能成熟到既功能强大又易于适配的程度。
- **引用边界**：这是 2004 年的领域判断，不是对今天的评价。写成"现有仿真器仍是 2D"会立刻被 2026 年的审稿人驳回；正确用法是"历史上仿真器的 2D 局限催生了 3D 多机器人仿真"。
- **用途**：论文"仿真平台"一节的背景句。

### Q18-2 · Gazebo 的定位：3D 动态多机器人环境

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[13_D_level_L18-L20.md](13_D_level_L18-L20.md)

- **拟写入论文**：Gazebo 以三维动态多机器人环境为目标，复现下一代移动机器人将面对的复杂世界。
- **来源**：`koenig2004design`｜**D 级（原文，见 D 级文件）**
- **原文**：
  > "Gazebo is designed to fill this niche by creating a 3D dynamic multi-robot environment capable of recreating the complex worlds that would be encountered by the next generation of mobile robots."
- **直译**：Gazebo 旨在填补这一空白，通过构建一个三维动态多机器人环境，能够复现下一代移动机器人将会遇到的复杂世界。
- **引用边界**：**这句只证明"Gazebo 提供三维动态环境"，完全不涉及流体、浮力、海流。** 论文里若写"Gazebo 提供了水动力仿真"，就是拿这句话去支撑它支撑不了的事。
- **用途**：说明"平台提供了什么"与"我们补了什么"的分界句。

### Q18-3 · LOTUSim 的多域定位

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[13_D_level_L18-L20.md](13_D_level_L18-L20.md)

- **拟写入论文**：LOTUSim 面向海洋机器人，支持空中—水面—水下的多域场景统一仿真。
- **来源**：`buche2026lotusim`｜**D 级（原文，见 D 级文件）**
- **定位**：摘要首段（multi-domain、marine robotics）；系统架构节。
- **检索锚点**：`multi-domain`、`marine robotics`、`UUV`、`USV`、`simulator`
- **引用边界**：**平台论文不等于验证论文**。它能支撑"该平台存在且面向多域"，不能支撑"该平台的动力学已被验证"。另外 2026 年 IROS 论文的元数据可能变动（源文件已标）。
- **待办**：无（已取证，见 D 级文件）

### Q18-4 · ROS 的价值在于可记录、可重放

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[13_D_level_L18-L20.md](13_D_level_L18-L20.md)

- **拟写入论文**：ROS 以节点与消息的松耦合组织机器人软件，并通过带时间戳的记录/重放支持实验复现。
- **来源**：`quigley2009ros`｜**D 级（原文，见 D 级文件）**
- **定位**：摘要/引言（pub-sub 架构）；工具链部分（bag、`use_sim_time`）。
- **检索锚点**：`publish/subscribe`、`message`、`node`、`rosbag`、`logging`
- **引用边界**：这是"架构能力"陈述，不是"复现性保证"——录包只记录消息，不记录代码版本与环境。把"录了包"等同于"可复现"是常见错误。
- **待办**：无（已取证，见 D 级文件）

---

## L19 验证与确认

### Q19-1 · 验证（verification）与确认（validation）是两个问题

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[13_D_level_L18-L20.md](13_D_level_L18-L20.md)

- **拟写入论文**：验证回答"模型是否被正确地建立"，确认回答"建立的是否为正确的模型"，二者判据不同。
- **来源**：`sargent2013verification`｜**D 级（原文，见 D 级文件）**
- **定位**：摘要 + 引言的定义段（conceptual validity / verification / operational validity 三词的定义）。
- **检索锚点**：`conceptual validity`、`operational validity`、`verification`、`validation`、`credibility`
- **为什么降级**：数据库返回的摘要缺失多处虚词（如 "described," 前后断句），直接引用会造出病句。
- **引用边界**：这对区分是**方法论**层面的，不提供量化门槛。因此不能用它论证"我们的模型已验证通过"——它只能论证"我们知道验证与确认不是一回事"。
- **待办**：无（已取证，见 D 级文件）

### Q19-2 · 数值误差要靠收敛性证据，而不是声明步长

> **状态**：⚠ 仅替代证据 · 证据：[13_D_level_L18-L20.md](13_D_level_L18-L20.md)

- **拟写入论文**：代码验证与解的验证要求给出离散误差估计（如网格/步长收敛），仅声明所用步长不足以证明数值解可信。
- **来源**：`oberkampf2010verification`（专著）｜**替代证据（Thacker 2004）**
- **定位**：代码验证章（manufactured solution、观测收敛阶）与解的验证章（Richardson 外推、GCI）。
- **检索锚点**：`code verification`、`method of manufactured solutions`、`order of accuracy`、`solution verification`、`grid convergence index`
- **引用边界**：该方法体系面向连续介质（CFD/结构），对含逻辑切换的控制系统（饱和、模式切换）适用性有限——这一点要在局限里写，否则会被问"你的步长收敛为什么在饱和时失效"。
- **待办**：抄 GCI 定义句或收敛阶定义句，升 D 级。这是论文"时间步长收敛实验"的方法依据。

---

## L20 可复现

### Q20-1 · 手工操作是不可复现性的源头

> **状态**：✅ 已取证 D 级（原文原句） · 证据：[13_D_level_L18-L20.md](13_D_level_L18-L20.md)

- **拟写入论文**：可复现要求数据、代码与计算环境同时可获取；任何手工操作环节都会破坏可复现性。
- **来源**：`sandve2013ten`（PLoS Comput Biol，开放获取）｜**D 级（原文，见 D 级文件）**；`peng2011reproducible`（Science 评论）｜**C 级（未单独取证）**
- **定位**：Sandve 十条规则的条目列表（尤其"原始数据不动""脚本化每一步""版本控制一切""记录软件版本"四条的标题句）；Peng 一文对"可重复 vs 可复现"的区分段。
- **检索锚点**：`reproducible`、`version control`、`script`、`raw data`、`provenance`
- **引用边界**：两篇都是**规范性**文献（该怎么做），不是实证（这样做有效）。引用它们是在声明研究规范，不是在证明结果可靠。
- **待办**：无（已取证，见 D 级文件）

---

## 本组提醒

- 本组 7 条里只有 2 条是 A 级（都来自 Koenig 2004），其余 5 条是规范/方法论，必须回原文补 D 级才有引用价值。
- **最该补的是 Q19-2**：论文里那句"我们做了步长收敛实验"，要么有 GCI 式的量化，要么就别写"验证"二字，只写"检查"。
- **最该克制的是 Q18-2**：它是全台账里最容易被拿去支撑错误主张的一句（"Gazebo 能仿真水动力"）。写完论文后建议专门搜一遍这句话用在哪。
