# UUV S4 功能展示指南

日期：2026-11-20  
用途：给导师/评审/同事演示“我们实现了什么”，以及每个功能对应的入口、运行代码和记录文件。  
配套总说明：[implementation_overview_20261120.md](implementation_overview_20261120.md)

---

## 1. 推荐演示入口

优先打开：

- [S4 joint dashboard](../artifacts/s4_joint_dashboard_20261120_01/index.html)
- [单轨迹交互查看器](../artifacts/s4_trajectory_viewer_20261120_01/index.html)

这两个都是自包含 HTML，双击即可，不需要 Python、服务器或网络。

---

## 2. 演示路线

### 5 分钟版

1. 打开 joint dashboard；
2. 看 18/18 热力表；
3. 点击 F2/P0、F2/P2 切换轨迹；
4. 切到 Repeatability；
5. 切到 Evidence，说明 45 / 5 / 18 分层；
6. 切到 GUI demo 看参照球静帧；
7. 打开六场景总览图收尾。

### 15 分钟版

1. 5 分钟版全部；
2. 打开 S1、S3/F2/P2、S4 三张场景图；
3. 展示 S4 A/B/C/D 路径对照；
4. 展示五条真实 pilot 记录；
5. 展示失败与门禁；
6. 展示 operational readiness 文档；
7. 最后跑一次全量测试。

---

## 3. 功能展示表

| # | 功能 | 演示什么 | 推荐操作 | 运行代码/入口 | 记录文件 |
|---|---|---|---|---|---|
| 1 | S1 基础闭环 | 配置、规划、控制、xdyn、Gazebo 回写能连通 | 打开场景图，指出起终点/障碍/实际轨迹/反馈链 | —（静态记录） | `artifacts/gui_reproduction_formal_20261115_01/S1/F1/P4` |
| 2 | S3/F2/P2 窄裕量避障 | 最紧名义场景真实闭环；净距约 0.916 m，裕量约 16 mm | 打开场景图，放大障碍区域，指出完整轨迹未隐藏 | — | `artifacts/gui_current_v1_20261118_01/S3/F2/P2` |
| 3 | S4 A/B/C/D 规划对照 | 找到路径不等于能稳定跟踪；几何影响跟踪/终点 | 打开 S4 规划图，比较 A/B/C/D 路径 | Python 记录；D 独立候选 | `artifacts/twin_axial_s4_planning_20261116_04`、`artifacts/twin_axial_s4_terminal_candidate_20261117_01` |
| 4 | D 五条真实 S4 pilot | F0/F1/F2 P0/P2/P4 真实 pilot 全部通过 | 展示 pilot 汇总表/acceptance，不把它说成 18 矩阵 | 复现入口：`artifacts/twin_axial_s4_terminal_real_pilot_20261118_01/run.py`（不建议现场跑） | `artifacts/twin_axial_s4_terminal_real_pilot_20261118_01` |
| 5 | 水流对照 | 常流 / 合成分段 / 实测输入与 P0/P2 响应 | 打开 current comparison dashboard | 可视化脚本：`scripts/visualize_current_comparison.py` | `artifacts/current_comparison_20261119_01` |
| 6 | 实测流 v10 | NOAA 6 样本、60–150 s onset、east-only | 展示 current profile 和 schedule 文件 | 数据/接口代码：`ros2/twin_axial_g12_current_v10`、`ros2/twin_axial_current_data_v1` | `artifacts/current_data_p1_20261118_01`、`artifacts/gui_current_v10_20261118_02` |
| 7 | S4 + measured-current 联合适配 | S4 D + v10 + G12 能通过 gate/trial | 打开 gate 记录，说明真实 trial 已启动 | 代码：`ros2/twin_axial_s4_current_v1`；gate 入口：`scripts/run_s4_current_joint_gate.sh` | `artifacts/s4_current_joint_20261120_02/gate/gate.json`、`artifacts/s4_current_joint_20261120_02/real/S4/F2/P0_v2` |
| 8 | Joint 18 矩阵 | 3 流 × 6 运行，18/18 通过 | 打开 dashboard 热力表，逐格点开轨迹 | 汇总：`scripts/summarize_joint18.py`；矩阵运行：`scripts/run_s4_joint18_matrix.sh`（复现，不建议现场跑） | `artifacts/s4_joint18_20261120_01` |
| 9 | Repeatability | 每个 initial 三条运行叠加；到达/净距/最大位置差 | dashboard → Repeatability，选 F2/P2 | 运行：`scripts/run_s4_joint18_second_repeats.sh`、`scripts/run_s4_joint18_control_second_repeats.sh`、`scripts/run_s4_joint18_third_repeats.sh` | `artifacts/s4_joint18_20261120_01/F0|F1|F2/*_repeat*` || 10 | 证据分层 | 45 joint / 5 real / 18 Python 不是同一个矩阵 | dashboard → Evidence | — | `artifacts/s4_joint18_20261120_01/joint_matrix_summary.json`、`twin_axial_s4_terminal_real_pilot_20261118_01`、`twin_axial_s4_terminal_validation_20261117_01` |
| 11 | 失败与门禁 | 失败、动态碰撞、门禁拒绝分开 | 打开失败门禁图，指出停止点/预检拒绝 | — | `artifacts/presentation_20261119_01/failure_and_gate.png`、`artifacts/gui_current_v2_20261118_03/S3/F2/P0`、`artifacts/twin_axial_g12_boundary_20261114_03/real_summary.json` |
| 12 | GUI demo + 参照球 | S4 场景、D 路径、v10 参照球；visualization-only | dashboard → GUI demo，或打开 `gui_demo_still.png` | 启动：`bash artifacts/s4_current_gui_demo_20261120_01/run.sh`；当前环境录制为黑帧，实际录屏用 Windows OBS/Game Bar | `artifacts/s4_current_gui_demo_20261120_01/demo.world`、`gui_demo_still.png` |
| 13 | 轨迹回放 | 一段快速打开、无计算的 UUV 轨迹 | 直接打开 viewer 或播放 `trajectory_reel.mp4` | 构建：`scripts/build_trajectory_viewer.py` | `artifacts/s4_trajectory_viewer_20261120_01/index.html`、`.mp4`、`.gif` |
| 14 | 六场景展示 | S1/S3/S4/水流/门禁/最终组合 | 打开总览图/回放 | 生成脚本：`scripts/build_presentation_scenes.py` | `artifacts/presentation_20261119_01` |
| 15 | simulator-only readiness | 能做什么、不能做什么 | 打开评审文档 | — | `docs/operational_readiness_20261120.md` |
| 16 | 全量测试 | 代码回归 | 运行 pytest | `python -m pytest -q` | 1256 passed |

---

## 4. 具体演示动作

### 4.1 18 矩阵

打开：

```text
artifacts/s4_joint_dashboard_20261120_01/index.html
```

操作：

1. 默认显示 Joint 18 matrix；
2. 指出 F0/F1/F2 三行；
3. 点击任意单元格，例如 `F2/P2`；
4. 右侧轨迹切换；
5. 说明单元格里是 `PASS + clearance + RMS`。

讲解词：

> 这是 3 流 × 6 运行的 joint 18 矩阵；18/18 全部通过。F0/F1 是 control，F2 才是 v10 measured-current。点击任意格可以看到对应记录轨迹。

### 4.2 Repeatability

操作：

1. 点击 `Repeatability` tab；
2. initial 选择 `F2/P2`；
3. 展示三条轨迹叠加；
4. 指出 arrival spread、clearance spread、max position diff；
5. 说明历史 P2 15.1 mm 与当前矩阵 0.11 mm 是两组数据，已分开标注。

### 4.3 证据分层

操作：

1. 点击 `Evidence layers`；
2. 指出 18/18、45 joint、5 real、18 Python；
3. 明确四者不是同一个矩阵。

讲解词：

> 45 条 joint 是 simulator joint；5 条是真实 S4 pilot；18 条是常量流 Python。它们不能互相替代，也不能合并成一个成功率。

### 4.4 GUI demo

操作：

1. 点击 `GUI demo`；
2. 展示 S4 障碍、D 路径、UUV 起点、v10 参照球；
3. 说明 visualization-only，不启动 xdyn/controller/gate/dynamics；
4. 说明球每 10 s 采样，间距/方向表示流速/流向。

注意：

- 不现场打开黑帧 `camera_demo.mp4`；
- 如果需要真正 Gazebo GUI，运行 `run.sh`，但当前环境的相机录制是黑帧。

### 4.5 单轨迹查看器

打开：

```text
artifacts/s4_trajectory_viewer_20261120_01/index.html
```

操作：

1. 自动播放；
2. 指出 UUV、障碍、D 路径；
3. 播放/暂停/重置；
4. 拖动时间轴；
5. 指出 clearance/path error 随轨迹变化。

---

## 5. 复现/运行命令

### 打开 dashboard / viewer

Windows：

```powershell
Start-Process "artifacts\s4_joint_dashboard_20261120_01\index.html"
Start-Process "artifacts\s4_trajectory_viewer_20261120_01\index.html"
```

WSL/Linux：

```bash
xdg-open artifacts/s4_joint_dashboard_20261120_01/index.html
xdg-open artifacts/s4_trajectory_viewer_20261120_01/index.html
```

### 重新生成 dashboard

```powershell
.\.venv\Scripts\python.exe scripts\build_joint_dashboard.py
```

### 重新生成单轨迹查看器

```powershell
.\.venv\Scripts\python.exe scripts\build_trajectory_viewer.py
```

### 汇总 joint 18

```powershell
.\.venv\Scripts\python.exe scripts\summarize_joint18.py
```

### 跑 joint gate

```bash
wsl.exe -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/scripts/run_s4_current_joint_gate.sh
```

### 跑单条 joint trial

```bash
wsl.exe -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/scripts/run_s4_current_joint_trial.sh
```

### 跑 joint 18 矩阵（复现用，时间长）

```bash
wsl.exe -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/scripts/run_s4_joint18_matrix.sh
```

### 跑 repeat

```bash
wsl.exe -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/scripts/run_s4_joint18_second_repeats.sh
wsl.exe -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/scripts/run_s4_joint18_control_second_repeats.sh
wsl.exe -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/scripts/run_s4_joint18_third_repeats.sh
```

### GUI demo

```bash
bash artifacts/s4_current_gui_demo_20261120_01/run.sh
```

### 全量测试

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

结果：`1256 passed`。

---

## 6. 记录文件速查

### 核心记录

| 记录 | 路径 |
|---|---|
| S1/F1/P4 | `artifacts/gui_reproduction_formal_20261115_01/S1/F1/P4` |
| S3/F2/P2 | `artifacts/gui_current_v1_20261118_01/S3/F2/P2` |
| S4 A/B/C | `artifacts/twin_axial_s4_planning_20261116_04` |
| S4 D candidate | `artifacts/twin_axial_s4_terminal_candidate_20261117_01` |
| D 五条真实 pilot | `artifacts/twin_axial_s4_terminal_real_pilot_20261118_01` |
| S4 Python 18 | `artifacts/twin_axial_s4_terminal_validation_20261117_01` |
| joint 18 | `artifacts/s4_joint18_20261120_01` |
| joint gate | `artifacts/s4_current_joint_20261120_02/gate/gate.json` |
| 早期 10 条 joint | `artifacts/s4_current_joint_20261120_02` |
| 水流数据 | `artifacts/current_data_p1_20261118_01` |
| v10 记录 | `artifacts/gui_current_v10_20261118_02` |

### 每条 trial 内部

| 文件 | 用途 |
|---|---|
| `acceptance.json` | 原始验收结果 |
| `navigation.jsonl` | 轨迹/clearance/path error/current |
| `state.jsonl` | 状态历史 |
| `commands.jsonl` | 控制命令 |
| `gazebo_poses.jsonl` | Gazebo 回写位姿 |
| `model.yml` | 本次模型/水流配置 |
| `plan.json` | 路径/场景 |
| `adapter_release.json` | 适配器释放绑定 |
| `gate.json` | 门禁证据（部分目录） |

---

## 7. 展示时建议强调的边界

### 可以强调

- 18/18 joint 矩阵通过；
- P0–P4 每个 initial 三条运行；
- 五条真实 S4 pilot；
- 18 条 Python 证据；
- gate/trial/acceptance 全链路；
- GUI demo 和轨迹回放；
- simulator-only operational readiness。

### 必须同时说明

- F0/F1 是 control，不是 measured-current；
- 五条真实 pilot ≠ 18 真实矩阵；
- 18 Python ≠ joint measured-current；
- 45 joint ≠ sea trial；
- 相机黑帧 MP4 不作为有效视频；
- 没有 operational/sea-trial certification。

---

## 8. 最短展示脚本

如果只有 2 分钟：

1. 打开 dashboard；
2. 指出 **18/18**；
3. 点击一个格子切换轨迹；
4. 切到 Evidence，指出 **45 / 5 / 18** 分层；
5. 切到 GUI demo，指出参照球是 visualization-only；
6. 结束。

一句话：

> 我们完成了 S4 多障碍下的规划—控制—xdyn—Gazebo 联合闭环，18/18 joint 矩阵和三轮 repeat 通过；展示用 dashboard 和轨迹回放，不依赖黑帧相机视频。
