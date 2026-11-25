# _supplements/ · 补充材料

## FossenHandbook（对应 L02-1）

L02-1《Handbook of Marine Craft Hydrodynamics and Motion Control》的**书本体 PDF 已入库**（`../L02-1_fossen2011handbook.pdf`，54 MB，正文命中 fossen / added mass）。这里是它的配套材料，来源是你给的仓库：<https://github.com/cybergalactic/FossenHandbook>。

| 内容 | 路径 | 说明 |
|---|---|---|
| 仓库快照 | `FossenHandbook/FossenHandbook-main/` | 仓库本身很轻（README + 封面 + LICENSE），**不是书的正文**；README 里是各章讲义的索引表 |
| Ch2 运动学 | `FossenHandbook/slides/Ch2_Kinematics.pdf` | 5.2 MB，坐标系、`η̇ = J(η)ν`、欧拉角/四元数 |
| Ch3 刚体动力学 | `FossenHandbook/slides/Ch3_RigidBodyKinetics.pdf` | 1.4 MB，`M_RB`、`C_RB`、反对称性 |
| Ch10 环境力 | `FossenHandbook/slides/Ch10_EnvironmentalForces.pdf` | 14.0 MB，**海流/相对速度 ν_r、风浪流载荷**——台账 Q02-2、Q07-1 的原文出处最可能在这一章 |
| Ch12 引导系统 | `FossenHandbook/slides/Ch12_GuidanceSystems.pdf` | 18.1 MB，LOS 引导、路径跟踪——台账 Q13-1 的出处 |

### 为什么有用

之前台账里 Q02-2（"海流只经相对速度进入"）、Q02-3（附加质量常数）、Q07-1（三条前提）都被标成 C 级"需回原文核对"，因为书里没有摘要可抓。**现在有了原书 PDF + Ch10 讲义**，这几条可以直接翻到对应小节抄原句，升到 D 级。

### 使用注意

- 仓库代码/README 为 MIT 许可；**讲义 PDF 是 NTNU 课程材料（TTK4190）**，仅作个人学习与引文核对，不要再分发。
- 讲义是"第 2/3 版"混编（Ch2/3/10 标注 3rd，Ch12 标注 2nd），与手上的书版次可能不完全对应；引用时以**书本体页码**为准，讲义只用来快速定位。

## 与台账的对应

| 台账条目 | 现在可以查的地方 |
|---|---|
| Q02-1 6-DOF 矢量形式 | 书 + Ch3 |
| Q02-2 / Q07-1 相对速度三前提 | 书 + Ch10 |
| Q02-3 附加质量常数 | 书 + Ch3 |
| Q13-1 LOS 几何 | 书 + Ch12 |
