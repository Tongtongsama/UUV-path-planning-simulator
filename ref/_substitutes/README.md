# _substitutes/ · 替代文献（20 篇，58.6 MB）

> **2026-11-25 20:30 更新**：`ref/` 现为 18/48，剩余 30 条全部是订阅墙。本目录覆盖其中 **24 条**（20 篇新下载 + 4 条用已入库文件顶替），**6 条无替代**。
>
> 注意：`SUB-L15-1_*.pdf` 对应的原文 L15-1 已由用户补齐，该替代文献保留作"运动动力学规划"背景阅读，不再是必需。

**这些不是原文。** 它们是为"拿不到原文时仍能读到同一技术内容"而挑的**开放获取**同类文献，全部按 `SUB-<缺失编号>_<作者><年>.pdf` 命名，`SUB-` 前缀就是防误用的标记。

## 使用铁律

| 能做 | 不能做 |
|---|---|
| 读它来理解某个概念、核对某个方法的机理 | 用它的页码/句子冒充原文引文 |
| 在论文里作为**补充/替代引用**（另起引文、说明理由） | 把 `SUB-` 文件当成 `ref/L13-2_*.pdf` 的替身，标注为 Lekkas 2014 |
| 用它交叉验证我对原文的判断 | 用它支撑"某作者说过 X"这类归属判断 |

一句话：**替代文献可以支撑"这个技术是这样的"，不能支撑"某某人在某年是这样写的"。**

## 对应表

| 缺失 | 替代文件 | 替代文献 | DOI | 覆盖什么 | 不覆盖什么 |
|---|---|---|---|---|---|
| L02-1 | `SUB-L02-1_chengsiongchin2026.pdf` | A Unified Dynamics Framework … Six-DOF Pipeline-Tracking ROV | 10.48550/arXiv.2610.04949 | 6-DOF 刚体+附加质量+阻尼的完整写法 | 不是 Fossen 手册，没有相对速度/海流的规范推导 |
| L03-1 | `SUB-L03-1_girishvithalraolakhekar2020.pdf` | Robust Diving Motion Control of an AUV（ANFIS 滑模） | 10.1109/ACCESS.2020.3001631 | 纵倾/下潜通道的建模与控制结构 | 无 Healey 的 12 状态解耦推导与 LOS 用法 |
| L03-2 / L13-1 / L13-2 / L13-3 | `SUB-L13-1_thoringefossen2014.pdf` | LOS Path Following for Dubins Paths with Adaptive Sideslip Compensation | 10.1109/TCST.2014.2338354 | LOS 几何、前视距离、侧滑/漂移补偿（一身兼四条） | 无 Lekkas 的样条参数化与积分 LOS 稳定性条件 |
| L04-1 | `SUB-L04-1_claudiopaliotta2018.pdf` | Trajectory Tracking and Path Following for Underactuated Marine Vehicles | 10.1109/TCST.2018.2834518 | 欠驱动跟踪/路径跟踪的现代表述 | 无 Aguiar 的参数自适应与可行性条件证明 |
| L04-2 | `SUB-L04-2_xiaogengliang2019.pdf` | Distributed and Self-Organized Swarm Control for Underactuated Marine Vehicles | 见 PDF 首页 | 欠驱动海洋载具的控制设计框架 | 侧重集群，非 Do & Pan 的单艇 backstepping 体系 |
| L05-1 | `SUB-L05-1_hiroshiuchihori2021.pdf` | LPV MPC of AUV for Docking Scenarios | 10.3390/app11104368 | 含推力分配约束的 AUV 控制实现 | 无 Johansen 的可达集/优先级分配理论 |
| L05-2 | `SUB-L05-2_tristnprez2009.pdf` | Constrained Control Design for DP with Control Allocation | 10.4173/mic.2009.2.2 | 分配 + 约束 + 优化的工程写法 | 面向 DP 船舶，非欠驱动 UUV |
| L06-1 | `SUB-L06-1_jeansmith2017.pdf` | Disturbance Observer Based Control with Anti-Windup（无人机） | 10.1007/s10846-017-0534-5 | 抗饱和的机制与接入闭环方式 | 对象是固定翼 UAV，不是海洋载具；无 LMI 理论 |
| L06-2 | `SUB-L06-2_pouriasarhadi2026.pdf` | Simple yet Effective Anti-windup Techniques … AUV Case Study | arXiv:2601.01302 | **AUV 专用**抗饱和（幅值+速率） | 无 Kim 2013 的变结构切换律 |
| L07-3 | `SUB-L07-3_thoringefossen2023.pdf` | Adaptive LOS Guidance Law for Path Following | 见 PDF 首页 | 漂移/海流下的自适应引导 | 不是观测器，不估计流速分量 |
| L08-2 | `SUB-L08-2_chinonsookereke2023.pdf` | Overview of ML Techniques in Local Path Planning for AUVs | 10.1109/ACCESS.2023.3249966 | AUV 规划的问题结构与方法谱系 | 无 Wang 2004 的大范围粗化策略 |
| L10-1 | `SUB-L10-1_dominiknu2018.pdf` | Random Finite Set Approach for Dynamic Occupancy Grid Maps | 10.1177/0278364918775523 | 占据栅格的现代概率表述 | 无 Elfes 1989 的原始贝叶斯更新推导 |
| L11-1 | `SUB-L11-1_haoyangfan2018.pdf` | Baidu Apollo EM Motion Planner | arXiv:1807.08048 | 路径平滑与优化后处理 | **不覆盖**快捷化与 clearance 论证 |
| L15-1 | `SUB-L15-1_mohamedelbanhawi2014.pdf` | Sampling-Based Robot Motion Planning: A Review | 10.1109/ACCESS.2014.2302442 | 运动学 vs 动力学约束的区分 | 无 Donald 1993 的完备性证明 |
| L15-2 | `SUB-L15-2_eliseishafer2026.pdf` | End-to-End Motion Planning for AUVs (RL) | arXiv:2606.08513 | AUV 上分层/端到端规划的近期做法 | 不是 Khorrami 的避障层级架构 |
| L16-1 | `SUB-L16-1_isahajimoh2024.pdf` | Tube-based MPC of an AUV using LOS Re-planning | 10.1016/j.oceaneng.2024.119688 | MPC 处理约束 + LOS 重规划的耦合 | 无 Shen 2020 的分布式求解 |
| L17-2 | `SUB-L17-2_madhusmitapanda2020.pdf` | A Comprehensive Review of Path Planning for AUVs | 10.1007/s11633-019-1204-9 | 规划算法全景（可替代 Kot 综述） | 无 Banik 的 min-max 鲁棒形式 |
| L18-1 | `SUB-L18-1_ikerzamora2016.pdf` | Extending OpenAI Gym … ROS and Gazebo | arXiv:1608.05742 | Gazebo 与 ROS 的集成方式 | 无 Koenig 2004 的仿真器架构论述 |
| L19-1 | `SUB-L19-1_benhthacker2004.pdf` | Concepts of Model Verification and Validation | 见 PDF 首页 | **verification/validation 概念定义**，与 Sargent 同源 | 无 Sargent 的仿真生命周期框架 |
| L20-2 | `SUB-L20-2_jeremygoecks2010.pdf` | Galaxy: accessible, reproducible computational research | 10.1186/gb-2010-11-8-r86 | 可复现研究的工程实现 | 不是 Peng 的 Science 评论 |

## 用现有文件顶替的（无需新下载）

| 缺失 | 用哪个已入库文件顶 | 理由 |
|---|---|---|
| L09-1 Hart 1968 | `../L08-1_lavalle2006planning.pdf` | LaValle 第 2–3 章完整讲 A*/Dijkstra/启发式与可采纳性 |
| L10-2 Latombe 1991 | `../L08-1_lavalle2006planning.pdf` | C-space、障碍膨胀、胞腔分解的标准表述 |
| L12-2 Yang 2010 | `../L12-3_park2022three.pdf` | 同为解析/连续曲率平滑，覆盖曲率界与重检问题 |
| L14-1 Sahoo 2019 | `../L14-2_bingul2023intelligent.pdf` | PID 类控制器的 AUV 实现与整定 |

## 仍未覆盖（6 条）

| 缺失 | 为什么没替代 |
|---|---|
| L01-1 Paull 2014 | 试过的最近结果是水下传感网络综述，主题不符，已删除 |
| L01-2 Sahoo 2019 | 综述类，OA 候选均不匹配 |
| L02-2 Antonelli 2018 | 专著；同一内容的 OA 章节未找到 |
| L04-3 Gao 2016 | 深度—纵倾内模控制，OA 候选匹配到不相关论文 |
| L07-2 Al Makdah 2019 | 海流补偿跟踪，候选与"current"只是词面匹配 |
| L19-2 Oberkampf 2010 | 专著；OA 候选匹配到医学指南，已删除 |

这 6 条建议直接走订阅：它们支撑的是论文里最"硬"的建模与验证依据，用二手替代反而不安全。

## 校验说明

每篇都过了三道：`%PDF` 魔数、>50 KB、正文（解压内容流抽文字）命中主题关键词。**但关键词只能证明主题相近，不能证明内容等价**——所以上表的"覆盖/不覆盖"是我逐篇看过标题与摘要后的判断，仍需你在使用时自行核对。
