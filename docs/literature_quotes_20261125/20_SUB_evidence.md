# 替代文献台账（SUB-）· 20 篇

2026-11-25。对象：[ref/_substitutes/](../ref/_substitutes/README.md)。

## 0. 先说明性质变化

这 20 篇是在"原文拿不到"的阶段下载的。**现在 `ref/` 已有 45/48 篇原文**，所以绝大多数 SUB 的定位从"证据来源"降级为"背景阅读"。真正还承担证据职能的只有 **1 篇**：

| 仍然必需 | 原因 |
|---|---|
| `SUB-L19-1_benhthacker2004.pdf`（Thacker 2004《Concepts of Model V&V》） | L19-2 Oberkampf & Roy 的书未获取，V&V 概念只能靠它 |

其余三条缺口（L10-2 Latombe、L14-1 Sahoo、以及 L19-2 的部分内容）用**已入库的原文**顶替，不需要 SUB：

| 缺失 | 用什么顶 |
|---|---|
| L10-2 Latombe 1991 | `../L08-1_lavalle2006planning.pdf`（C-space、Minkowski、胞腔分解） |
| L14-1 Sahoo 2019 OMAE | `../L14-2_bingul2023intelligent.pdf`（PID 类 AUV 控制） |

## 1. 证据强度分级

| 档 | 含义 | 可否引用 |
|---|---|---|
| **A** | 摘要/正文有可逐字引用的句子 | 可以（引替代文献本身） |
| **B** | 只有标题与摘要大意，没有干净的原句 | 只能当背景，不能引用 |
| **C** | 文本层损坏（移位编码/仅公式），无法抽句 | 不能引用 |

## 2. 台账

### A 档（有可引用原句）

#### SUB-L02-1 · Chin et al. 2026（代 L02-1 Fossen 手册）

> “This paper reports a pipeline-tracking control architecture for a six-degree-of-freedom remotely operated vehicle (ROV) in which a single Universal Scene Description (USD) scene supplies th[e dynamics] …”

- 能支撑：**6-DOF 载具模型**在仿真中需要刚体+水动力+推进的完整描述；"同一场景供动力学"这一工程做法。
- 不能支撑：Fossen 的矢量模型标准形式、`ν_r` 定义、附加质量常数假设——这些仍须书。
- 用途：模型章"6-DOF 需要哪些项"的旁证。

#### SUB-L03-1 · Lakhekar et al. 2020（代 L03-1 Healey）

> “In the derivation of diving motion equations of an AUV, the pitch angle of the vehicle is often assumed to be small in the vertical plane. This is a quite strong restricting condition in underwater operations and may cause serious modeling inaccuracies in AUV's dynamics.”

- 能支撑：**"小纵倾假设"是强约束、会带来建模误差**——这正是本项目必须声明降维理由的直接依据。
- 不能支撑：Healey 的 12 状态解耦模型与滑模设计。
- 用途：模型章"降维假设的代价"；比泛泛说"小角度近似"有力得多。

#### SUB-L05-2 · Pérez & Donaire 2009（代 L05-2 Fossen & Johansen 2006）

> “In this paper, we address the control design problem of positioning of over-actuated marine vehicles with control allocation. The proposed design is based on a combined position and velocity loops in a multi-variable anti-windup implementation together with a control allocation mapping.”

- 能支撑：**分配与抗饱和必须联合设计**（同一篇里同时出现 anti-windup 与 allocation），这是 L05/L06 交叉的现成证据。
- 不能支撑：分配的可达集理论、推力损失。
- 用途：讨论"分配层与抗饱和的耦合"时引用。

#### SUB-L06-1 · Smith et al. 2017（代 L06-1 Tarbouriech）

> “An accurate mathematical model is unlikely to be available meaning optimal control methods become difficult to apply. Furthermore, their reduced weight and inertia mean they are significantly more vulnerable to environmental disturbances such as wind gusts.”

- 能支撑：**模型不准 + 扰动大 ⇒ 需要抗饱和/扰动观测**这一动机链。
- 不能支撑：LMI 综合、AW 补偿器的稳定性理论；对象是无人机不是 UUV。
- 用途：抗饱和小节的动机句（并注明对象差异）。

#### SUB-L07-3 · Fossen, Pettersen & Galeazzi 2023（代 L07-3 Kim 2018）

> “Despite the effectiveness and popularity of proportional LOS guidance laws, they have limitations when the vehicle is exposed to drift forces caused by wind, waves, and ocean currents.”

- 能支撑：**比例 LOS 在漂移力（含海流）下失效**——这正是本项目要考察"恒流下 LOS+PID 表现"的动机。
- 不能支撑：观测器设计、流估计误差分析。
- 用途：引导章动机句；比引 2014/2015 两篇更贴"比例 LOS 的局限"这一具体点。

#### SUB-L10-1 · Nuss et al. 2018（代 L10-1 Elfes）

> “Grid mapping is a well established approach for environment perception in robotic and automotive applications. Early work suggests estimating the occupancy state of each grid cell in a robo[t] …”

- 能支撑：占据栅格作为"既成方法"的地位与基本假设。
- 不能支撑：Elfes 1989 的贝叶斯更新推导（原文已有，应引原文）。
- 用途：只在需要"现代综述视角"时作补充引用。

#### SUB-L19-1 · Thacker et al. 2004（**仍是唯一支撑**，代 L19-2 Oberkampf）

> “Model veriﬁcation and validation (V&V) is an enabling methodology for the development of computational models that can be used to make engineering predictions with quantiﬁed conﬁdence. Model V&V procedures are needed by government and industry to reduce the time, cost, and risk associated with full-scale testing …”

- 能支撑：V&V 的目的（带量化置信度的预测）、为何需要 V&V（替代全尺寸试验）。
- 不能支撑：Oberkampf 的三层框架（代码验证 / 解的验证 / 确认）与验证层级——那是该书特有的组织方式。
- 用途：验证章的"为什么要做 V&V"；写"代码验证/解的验证"时须另找出处或改写为"计算验证"。

#### SUB-L20-2 · Goecks et al. 2010（代 L20-2 Peng）

> “Galaxy … automatically tracks and manages data provenance and provides support for capturing the context and intent of computational methods.”

- 能支撑：**provenance（来源追溯）是可复现性的工程抓手**——不只是"存代码"，还要记录"数据从哪来、怎么算的"。
- 不能支撑：Peng 关于"可重复 vs 可复现"的区分。
- 用途：实现章的"日志/溯源设计"。

### B 档（仅标题级，无干净原句）

| 文件 | 代谁 | 能支撑的大意 | 不能支撑 |
|---|---|---|---|
| `SUB-L04-2_xiaogengliang2019.pdf` | L04-2 | 欠驱动海洋载具的（集群）控制框架 | Do & Pan 的 backstepping 体系 |
| `SUB-L08-2_chinonsookereke2023.pdf` | L08-2 | AUV 局部规划的方法谱系、海流建模的重要性 | 大范围图搜索与粗化 |
| `SUB-L11-1_haoyangfan2018.pdf` | L11-1 | 路径后处理中的平滑与优化 | **快捷化与 clearance 论证** |
| `SUB-L13-1_thoringefossen2014.pdf` | L13-1/2/3 | Dubins 路径 + 侧滑补偿的 LOS | 样条参数化、积分律稳定性 |
| `SUB-L15-1_mohamedelbanhawi2014.pdf` | L15-1 | 采样规划综述，含运动学/动力学约束区分 | Donald 的完备性证明 |
| `SUB-L15-2_eliseishafer2026.pdf` | L15-2 | AUV 端到端（RL）规划—执行一体化的近期尝试 | Khorrami 的 WAP/LAP 分层 |
| `SUB-L16-1_isahajimoh2024.pdf` | L16-1 | MPC + LOS 重规划的耦合 | Shen 的分布式求解 |
| `SUB-L17-2_madhusmitapanda2020.pdf` | L17-2 | AUV 规划算法全景 | Banik 的 min-max 鲁棒形式 |
| `SUB-L18-1_ikerzamora2016.pdf` | L18-1 | ROS 与 Gazebo 的集成范式 | Koenig 2004 的仿真器架构论述 |

### C 档（文本层损坏，无法引用）

| 文件 | 代谁 | 问题 |
|---|---|---|
| `SUB-L04-1_claudiopaliotta2018.pdf` | L04-1 | 文本层是移位编码（`7 U D M H F W R U \`），抽出来是乱码；文件本身可正常阅读 |
| `SUB-L06-2_pouriasarhadi2026.pdf` | L06-2 | 只有 1480 字符且几乎全是公式，无可用散文句 |

## 3. 使用规则

1. **SUB 不能冒充原文**：引用时必须写清"见 Thacker et al. (2004)"，不能标成 Oberkampf & Roy 或 Fossen。
2. **A 档才能进正文**；B 档只用于"我知道有这么一类工作"的铺垫；C 档只能人工翻阅。
3. **原文已在库的，一律优先引原文**。现在只剩 `SUB-L19-1` 是必需项，其余 19 篇属于可选背景。
4. 若后续补齐 L19-2 原书，`SUB-L19-1` 也应降级为背景。

## 4. 与本台账其余部分的关系

| 文件 | 作用 |
|---|---|
| [10](10_D_level_L02-L07.md) ~ [13](13_D_level_L18-L20.md) | **原文**逐条取证（37 条） |
| 本文件 | **替代文献**的可用边界（20 篇，1 篇仍必需） |
| [99](99_claim_index.md) | 47 条断言的总账与状态 |

一句话：**原文负责"作者到底说了什么"，替代文献只负责"这个技术本身大概是什么"。两者不能互换。**
