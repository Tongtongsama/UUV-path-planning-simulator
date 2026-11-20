# UUV S4 联合场景实现说明

日期：2026-11-20  
范围：**simulator-only**；含 S4 多障碍规划、G12 控制、xdyn 积分、Gazebo 回写、实测流输入、验收门禁、证据保存与展示包。  
不包含：真实海试、实艇部署、operational certification。

---

## 1. 这份文档说明什么

这份文档回答三个问题：

1. 我们实现了什么；
2. 这些内容如何组成一条可复现的闭环；
3. 目前哪些结论有证据、哪些边界不能越过。

核心结论：

> 系统已经完成 S4 多障碍场景下的规划—控制—xdyn—Gazebo—反馈闭环；完成 S4 联合 18 工况矩阵和 P0–P4 三轮 repeat；保留五条真实 S4 pilot 与 18 条 S4 Python 证据；完成展示 dashboard、轨迹回放和 simulator-only 运行就绪评审。

---

## 2. 系统实现总览

| 层次 | 实现内容 | 主要证据 |
|---|---|---|
| 场景 | S1、S3、S4 多障碍地图 | `ros2/twin_axial_feasible_matrix/scenes.py`、S4 world |
| 规划 | A*、安全捷径、圆角/平滑、4 m 终点段 | `artifacts/twin_axial_s4_planning_20261116_04` |
| 控制 | 冻结 G12/v6 制导、PID、分配器 | `ros2/twin_axial_g12_link_v6` |
| 动力学 | xdyn 6DOF 积分 | 各 trial 的 `xdyn.log`、`state.jsonl` |
| 仿真 | Gazebo/LOTUSim 场景回写与位姿同步 | `gazebo_poses.jsonl` |
| 水流 | 常流、合成分段流、NOAA 实测流 | `artifacts/current_data_p1_20261118_01` |
| 联合适配 | S4 D path + v10 measured-current | `ros2/twin_axial_s4_current_v1` |
| 门禁 | 传输、运行时、Python 证据、释放绑定 | `gate.json`、`adapter_release.json` |
| 验收 | 原始 acceptance、失败/拒绝分离 | `acceptance.json`、`failure.json` |
| 展示 | dashboard、轨迹查看器、场景图、回放 | `artifacts/s4_joint_dashboard_20261120_01` |
| 评审 | simulator-only readiness | `docs/operational_readiness_20261120.md` |

---

## 3. 路径规划：S4 A/B/C/D

S4 是五障碍地图，起点约 `(2,-10)`，目标 `(18,-10)`。规划对照包含四种路径：

| 方法 | 含义 | 作用 |
|---|---|---|
| A：raw A* | 原始 A* 网格路径 | 基线 |
| B：safe shortcut | 安全捷径/共线点消除 | 减少不必要的折返 |
| C：rounded / smoothing | 安全圆角/平滑 | 降低跟踪突变 |
| D：4 m terminal segment | 独立候选，最后 4 m 水平终点段 | 改善终点接近与保持 |

边界：

- A/B/C 是 **F0/P0 Python 消融**；
- D 是独立候选；
- D 有五条真实 S4 pilot（F0/F1/F2 P0/P2/P4）；
- D 的真实 pilot 不是 18 真实矩阵。

展示：

- [S4 planning comparison](../artifacts/presentation_20261119_01/s4_planning_comparison.png)
- [S4 Python 18 记录](../artifacts/twin_axial_s4_terminal_validation_20261117_01)

---

## 4. 控制与动力学闭环

G12 控制链保持不变：

```text
路径/制导
  → G12 guide
  → PID + 推力分配
  → xdyn 6DOF 积分
  → Gazebo/LOTUSim 位姿回写
  → 状态反馈
```

实现要点：

- 控制器参数冻结，不在 trial 中调参；
- 推力分配保留上下推进器约束；
- xdyn 使用完整 6DOF，不做状态裁剪；
- Gazebo 只做场景与位姿回写，不替代动力学；
- 每条 trial 保存：
  - `navigation.jsonl`
  - `state.jsonl`
  - `commands.jsonl`
  - `gazebo_poses.jsonl`
  - `acceptance.json`

S1 基础闭环示例：

- 场景：S1/F1/P4
- 到达：74.85 s
- 净距：1.62944 m
- 证明：配置、规划、控制、xdyn、Gazebo 回写能连通。

展示：

- [S1 basic closed loop](../artifacts/presentation_20261119_01/s1_basic_closed_loop.png)

---

## 5. 水流输入：常流、合成流、实测流

### 5.1 常流 F2

- `(x,z) = (0, 0.05) m/s`
- 用于历史基线与控制对照。

### 5.2 合成分段流 v4

- x 分量分段变化；
- z=0.05；
- 用于验证时变流输入接口。

### 5.3 NOAA 实测流 v10

- 数据源：NOAA CO-OPS `pm0101` bin 1；
- 窗口：2026-11-17 05:15–05:45 UTC；
- 6 个样本，重采样到 0/30/60/90/120/150 s；
- east 分量：约 `0.0325 → -0.3146 m/s`；
- onset：60–150 s；
- 控制器在 F2 下使用 v10 schedule；
- 北向分量仅保留在 metadata，不进入二维 `(x,z)` 修正。

展示：

- [current comparison dashboard](../artifacts/current_comparison_20261119_01/comparison_dashboard.png)
- [current profiles](../artifacts/current_comparison_20261119_01/current_profiles.png)

边界：

- 三种输入不同，不能据此做控制器优劣排名；
- 实测数据只有单站 6 样本，east-only；
- 没有垂直流/剪切。

---

## 6. 联合适配：S4 D + v10 measured-current

新增 `ros2/twin_axial_s4_current_v1`：

- S4 D 路径；
- F0：无流；
- F1：常流；
- F2：v10 measured-current；
- G12 控制器与 S4 场景保持；
- F0/F1 只作 control；
- F2 才是 measured-current。

联合门禁检查：

- v6 传输 probes；
- 运行时环境；
- S4 Python 证据；
- v10 measured-current Python 证据；
- release/source 绑定。

结果：

- gate `passed=true`
- `ready_to_run=true`
- 正式 trial 启动前必须通过该 gate；
- 失败/拒绝记录单独保存。

---

## 7. Joint 18 矩阵与 repeat

### 7.1 矩阵定义

3 个 flow × 6 个运行：

```text
F0: P0 P1 P2 P3 P4 P0_repeat   (no-current control)
F1: P0 P1 P2 P3 P4 P0_repeat   (constant-current control)
F2: P0 P1 P2 P3 P4 P0_repeat   (v10 measured-current)
```

### 7.2 结果

- **18/18 通过**；
- 全部原始 acceptance checks true；
- 到达时间：60.45–61.35 s；
- 最小净距：1.4415–1.5696 m；
- 安全裕量：0.5415–0.6696 m；
- RMS：0.1325–0.1672 m；
- 最大误差：0.3802–0.4020 m。

### 7.3 Repeat

为覆盖 P0–P4 的稳定性，继续补跑到 **每个 initial 3 条运行**：

- 18 矩阵；
- 15 条第二轮 repeat；
- 12 条第三轮 repeat；
- 合计 **45 条 joint 记录**。

repeatability 观察：

- 多数 initial 的最大位置差在亚毫米到 0.15 mm；
- 早期独立 10 记录集合中 P2 曾出现 **15.1 mm**；
- 当前 18 矩阵 P2 三次运行的 spread 是 **0.11 mm**；
- 两者在文档和 dashboard 中分开标注，不能混写。

汇总：

- [joint_matrix_summary.json](../artifacts/s4_joint18_20261120_01/joint_matrix_summary.json)
- [JOINT18.md](../artifacts/s4_joint18_20261120_01/JOINT18.md)
- [STATUS.md](../artifacts/s4_joint18_20261120_01/STATUS.md)

边界：

- F0/F1 是 control，不是 measured-current；
- 45 joint 不是 18 真实矩阵；
- 45 joint 不是 sea trial；
- 18 矩阵不是 operational certification。

---

## 8. 五条真实 pilot 与 18 条 Python 的保留

### 五条真实 S4 pilot

`artifacts/twin_axial_s4_terminal_real_pilot_20261118_01`

- F0/P0
- F1/P0
- F2/P0
- F2/P2
- F2/P4

全部通过。  
它们是真实 S4 pilot，但不是 18 真实矩阵。

### 18 条 S4 Python

`artifacts/twin_axial_s4_terminal_validation_20261117_01`

- `all_18_original_passed=true`
- `all_18_verified=true`
- 三组 repeat 通过

它们是常量流 F0/F1/F2 的 Python 证据，不是 joint measured-current。

### 证据分层

```text
45 joint records   ≠  5 real pilots   ≠  18 Python   ≠  sea trial
```

---

## 9. 失败与门禁

系统没有把失败隐藏成成功：

- 历史 v2 失败轨迹停在记录停止点；
- 强流 U10/U15/U20：`real_started=false`，`python_gate_refusal`；
- 早期 S4 full gate 拒绝：`formal_trials_started=false`；
- 失败、动态碰撞、门禁拒绝分开表示。

展示：

- [failure and gate](../artifacts/presentation_20261119_01/failure_and_gate.png)

---

## 10. 展示包

### 10.1 主 dashboard

[**S4 joint dashboard**](../artifacts/s4_joint_dashboard_20261120_01/index.html)

- 3×6 热力表：F0/F1/F2 × P0–P4 + P0 repeat；
- 每格 PASS + clearance + RMS；
- 点击切换轨迹；
- repeatability：P0–P4 三条运行叠加；
- 证据分层：45 joint / 5 real / 18 Python；
- GUI 静帧说明。

### 10.2 轨迹查看器

[trajectory viewer](../artifacts/s4_trajectory_viewer_20261120_01/index.html)

- 单条 S4/F2/P0 轨迹；
- UUV 朝向、clearance、error、current；
- 播放/暂停/速度/时间轴；
- 纯前端，无计算。

### 10.3 场景图

- [S1](../artifacts/presentation_20261119_01/s1_basic_closed_loop.png)
- [S3/F2/P2](../artifacts/presentation_20261119_01/s3_narrow_margin.png)
- [S4](../artifacts/presentation_20261119_01/s4_planning_comparison.png)
- [水流对照](../artifacts/current_comparison_20261119_01/comparison_dashboard.png)
- [失败与门禁](../artifacts/presentation_20261119_01/failure_and_gate.png)
- [最终样本组合](../artifacts/presentation_20261119_01/final_sample_fusion.png)

### 10.4 GUI 静帧

[gui_demo_still.png](../artifacts/s4_trajectory_viewer_20261120_01/gui_demo_still.png)

明确标注：

- visualization-only；
- 不启动 xdyn/controller/gate/dynamics；
- 参照球每 10 s 采样，间距/方向表示流速/流向；
- 不是相机视频。

### 10.5 回放视频

- [presentation_reel.mp4](../artifacts/presentation_20261119_01/presentation_reel.mp4) — 六场景展示
- [trajectory_reel.mp4](../artifacts/s4_trajectory_viewer_20261120_01/trajectory_reel.mp4) — 单轨迹
- [trajectory_reel.gif](../artifacts/s4_trajectory_viewer_20261120_01/trajectory_reel.gif)

---

## 11. 相机/GUI 视频的限制

已实现相机录制管线：

- SDF camera sensor；
- `CameraVideoRecorder`；
- `/record_video` service；
- 可生成 1280×720 MP4。

但当前环境：

- EGL 无法创建 dri2 screen；
- 抽帧纯黑；
- 尝试 ogre2、ogre、软件 Mesa、Xvfb，仍黑帧。

因此：

- 相机 MP4 **不作为有效证据**；
- 展示使用轨迹查看器 + GUI 静帧；
- 真正的 GUI 视频需要在有可用 EGL/GPU 的环境，或 Windows 侧 OBS/Game Bar 录屏。

---

## 12. simulator-only 运行就绪评审

文档：

- [operational_readiness_20261120.md](operational_readiness_20261120.md)

结论：

| 维度 | 状态 |
|---|---|
| 配置 | simulator ready |
| 规划 | demonstrated |
| 控制 | demonstrated |
| 集成 | demonstrated |
| repeatability | demonstrated in sim |
| 边界/门禁 | demonstrated |
| GUI 视频 | not accepted |
| sea trial | not attempted |

严格结论：

> 可以做 simulator-level demonstration/review；不是 operational deployment，不是 sea-trial certification。

---

## 13. 复现与打开方式

### 主 dashboard

```text
直接打开：
artifacts/s4_joint_dashboard_20261120_01/index.html
```

### 轨迹查看器

```text
直接打开：
artifacts/s4_trajectory_viewer_20261120_01/index.html
```

### GUI demo

```bash
bash artifacts/s4_current_gui_demo_20261120_01/run.sh
```

### 全量测试

```bash
python -m pytest -q
```

结果：**1256 passed**。

---

## 14. 文件索引

| 类别 | 路径 |
|---|---|
| 主 dashboard | `artifacts/s4_joint_dashboard_20261120_01/index.html` |
| 单轨迹 viewer | `artifacts/s4_trajectory_viewer_20261120_01/index.html` |
| joint 18 汇总 | `artifacts/s4_joint18_20261120_01/joint18_summary.json` |
| joint 总汇总 | `artifacts/s4_joint18_20261120_01/joint_matrix_summary.json` |
| joint 状态 | `artifacts/s4_joint18_20261120_01/STATUS.md` |
| 五条真实 pilot | `artifacts/twin_axial_s4_terminal_real_pilot_20261118_01` |
| 18 条 Python | `artifacts/twin_axial_s4_terminal_validation_20261117_01` |
| 场景图 | `artifacts/presentation_20261119_01` |
| 水流对照 | `artifacts/current_comparison_20261119_01` |
| GUI demo | `artifacts/s4_current_gui_demo_20261120_01` |
| 运行就绪评审 | `docs/operational_readiness_20261120.md` |
| 展示包说明 | `artifacts/presentation_20261119_01/SCENES.md` |

---

## 15. 未完成与不能声称的内容

未完成：

- 可用（非黑帧）Gazebo 相机/GUI 视频；
- 真实海试 / operational certification；
- 更完整的硬件和环境边界验证。

不能声称：

- 45 joint = 18 真实矩阵；
- 五条真实 pilot = 18 真实矩阵；
- 18 Python = joint measured-current；
- F0/F1 = measured-current；
- simulator-only = sea-trial；
- 相机黑帧 MP4 = 有效视频证据。

---

## 16. 总结

项目已经形成一条可复现、可审计的仿真闭环：

```text
S4 场景
  → A/B/C/D 规划
  → G12 控制
  → xdyn 6DOF
  → Gazebo 回写
  → 实测流输入
  → gate/trial
  → 18 矩阵 + repeat
  → acceptance/失败记录
  → dashboard/轨迹回放/评审
```

当前展示内容与证据边界已经一致：  
**能证明的写清楚，不能证明的明确标出。**
