# 第二章（Literature Review）写作说明

日期：2026-11-27。对应正文：[chapter2_literature_review.md](chapter2_literature_review.md)。本文件是写作说明，不进论文正文。

## 1. 字数

正文（不含节标题与参考文献）共 **3,407** 词；各节均落在工作大纲给的区间内：

| 节 | 词数 | 大纲区间 |
|---|---:|---|
| 章首导语 | 74 | — |
| 2.1 | 339 | 250–350 |
| 2.2 | 454 | 400–500 |
| 2.3 | 437 | 400–500 |
| 2.4 | 405 | 400–500 |
| 2.5 | 550 | 450–550 |
| 2.6 | 498 | 400–500 |
| 2.7 | 426 | 350–450 |
| 2.8 | 224 | 150–250 |

若导师要求压到 3,000 左右，最先可删的三处是：2.5 中 Park (2022) 的三维平滑一句（−10）、2.6 中无模型 iPID 的插入语（−25）、2.6 中"从自身运动估计海流"一句（−17），合计约 −52；再删 2.2 关于附加质量常数化的说明（−30）即可接近 3,300。

## 2. 每节 → 文献 → 段落功能

| 节 | 主要来源（L 编号 / bib key） | 该节承担的论证功能 |
|---|---|---|
| 2.1 | L01（paull2014auv, sahoo2019advancements, wynn2014auvs）；L08-1（lavalle2006planning） | 把任务收窄为"已知静态图上的点到点转移"；声明定位与建图为输入；给出规划问题的形式定义与排除项 |
| 2.2 | L02（fossen2011handbook, antonelli2018underwater）；L03（healey1993multivariable, lapierre2007nonlinear）；L04（aguiar2007trajectory） | 6-DOF 各项作用 → 纵向降维的文献依据与"设计用降维、积分用 6-DOF"的区分 → `τ=Bu` 的秩与无直接升沉力 |
| 2.3 | L05（johansen2013control, fossen2006survey）；L06（tarbouriech2009antiwindup, kim2013variable）；L07（fossen2011handbook, kim2018observer） | 相对速度三条前提 → 分配的可达集与低冗余直接反解 → 包络、积分饱和、抗饱和的边界（不借稳定性证明） |
| 2.4 | L08-1；L09（hart1968formal, dechter1985generalized）；L10（elfes1989using, latombe1991robot）；L08-2 仅 future work | 离散图 + 代价 + 启发式三者的最优性条件；膨胀是设计约束而非物理保证；确定性栅格 ≠ 概率建图 |
| 2.5 | L11（geraerts2007creating）；L12（fraichard2004reeds, yang2010analytical, park2022three）；L15（donald1993kinodynamic, khorrami2009hierarchical） | 折线为何不可跟随 → 捷径必须逐段重检 → 圆角≠连续曲率 → 几何指标与闭环指标两组，导出 S4（A/B/C/D）的动机 |
| 2.6 | L13（fossen2003line, lekkas2014integral, fossen2015direct）；L14（sahoo2019development, bingul2023intelligent）；L16 各一句 | LOS 几何 + 前视权衡 + 投影/参考模型 → 引导与控制的界线、指令经分配后成为饱和闭环 → 终点捕获；明确不与积分 LOS 等价 |
| 2.7 | L18（koenig2004design, buche2026lotusim, quigley2009ros）；L19（sargent2013verification, oberkampf2010verification）；L20（sandve2013ten, peng2011reproducible）；L21、L22 | 平台能力 vs 本项目实际使用 → verification 与 validation 分层 → 冻结、哈希、失败保留；单站流只锚定量级 |
| 2.8 | 综合 | 定位句（连接而非发明）+ RQ1–RQ3，与 `dissertation_scope_baseline_v1.md` 一致 |

## 3. 提交前建议补证或改写的 6 处

按"风险从高到低"排列。这些不是文字问题，是证据问题。

1. **Fossen (2011) 的三条海流前提**（2.3 第 1 段）。"恒定、无旋、空间均匀"以及"恒定流本身不破坏相对运动稳定性、非对称载具留下常值偏置"这两步推论，属笔记的 ○ 级；Fossen 手册 PDF 无文本层，台账 Q02-2/Q07-1 仍是 C 级。这是 2.3 全节的承重点，建议从手册对应节抄 1 句原文再做标注。
2. **Khorrami & Krishnamurthy (2009) 的层间接口**（2.5 第 4 段）。正文写成"接口是航路点列表 + 速度指令"，属 C 级推断。若原文架构图不是这个形式，应改为"该架构中高层只交出任航路点序列"。
3. **Sahoo et al. (2019b, OMAE)**（2.6 第 2 段）。全文未获取（台账 Q14-1 仍为 ⏳）。正文只用了"分通道 PID + 带宽分离是小型 AUV 通行做法"这一层，未引任何数字；若审稿追问，可改引 L01-2 的综述或删去带宽 5–10 倍的暗示（正文已未写具体倍数）。
4. **LOTUSim (2026)**（2.7 第 1 段）。camera-ready 元数据未确认（源文件第 4 节第 7 条）。正文只用"multi-domain simulator … interfaces to external dynamics"这一层，未写 Ekman 分层流被验证，也未写其水动力精度。
5. **Geraerts & Overmars (2007)**（2.5 第 2 段）。"捷径可能更贴近障碍"是 B 级摘要 + ○ 级推导，正文已写成条件句（"may pass closer"），风险低；如要写强，需补原文的 clearance 数据。
6. **Donald et al. (1993)**（2.5 第 4 段）。扫描版无法机读，正文只用了"几何可行 ≠ 动力学可行"这一命题，未写复杂度类，属安全用法。

已按引文台账更正、正文已采用的口径：Yang & Sukkarieh 为参数化三次 Bézier 且 G1 不保证曲率连续；Kim et al. (2013) 为 dual-loop VSPID + 抗饱和、真艇 Cyclops 验证；Lekkas & Fossen 的积分律原文限定直线路径；Fossen & Lekkas (2015) 限定水平面运动。

## 4. 与现有草稿的冲突（需先处理，否则第 2 章与第 4 章互相矛盾）

1. `docs/dissertation/draft_v1.md` 第 74 行与第 177 行写"本文**不做**捷径化与平滑，只保留去重与共线点消除"。S4 之后这一句已不成立：S4 的 B 是安全捷径、C 是安全圆角、D 是捷径 + 水平终点段。新第 2.5 节按 S4 的实际做法写（"safe shortcut + 圆角 + 终点段"，且明确圆角不等于连续曲率）。**这两处旧文必须改**，否则第 4 章的方法描述会与第 2 章的文献定位冲突。
2. `draft_v1.md` 的第 2 章是中文五小节（2.1 建模 / 2.2 规划 / 2.3 引导 / 2.4 饱和 / 2.5 仿真），与本次八节英文结构不同，且引用为数字编号 [n]。建议整章替换，不要两套第二章并存；若学院要求中文正文，可按本稿逐段回译，结构与引文不动。
3. 章号引用已按 `draft_v1.md` 的九章结构写死：B 矩阵与包络 → 第 3 章；分配律 → 第 4 章；S4 的 A/B/C/D 结果 → 第 7 章；局限 → 第 8 章。若后续章节重排，只需改这四处。
4. `draft_v1.md` 摘要仍写"不做平滑"，与 2.5 冲突，同样需要同步。

## 5. 刻意没有写进正文的内容（边界）

- RRT*/APF/ACO 只字未提：本项目未实现，写了会被读成"综述过但没用"。
- MPC/SMC 各一句，且明说未做同条件比较、不排名。
- 三维平滑只作为 L12 的一条边界，不展开。
- 积分 LOS 明确"未实现、不等价"，避免把 PID 的积分说成积分 LOS。
- 占据栅格只用于"离散表示"，不声称概率建图或 SLAM。
- 没有出现任何具体参数（前视 2.3 m、膨胀 0.9 m、±15 N、|X|+4|M|≤30、54 条矩阵），全部留给第 3–7 章。

## 6. 参考文献

正文共引 42 条，全部来自 `docs/literature_review_sources_20261125.bib` 的 key，采用 author–year，附 DOI；已核对"每条都在正文出现、正文每个引用都有条目"。提交前需按学院模板转为数字编号或特定期刊格式（`docs/dissertation_draft_20261116/manuscript_outline.md` 记录该格式尚未确定）。
