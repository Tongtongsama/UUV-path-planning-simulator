# LOTUSim 最小动力学闭环里程碑封存

- 里程碑 ID：`lotusim-minimal-dynamics-20261028`
- 封存日期：2026-10-28
- 证据索引：[`artifacts/milestones/lotusim_minimal_dynamics_20261028/evidence_index.json`](../artifacts/milestones/lotusim_minimal_dynamics_20261028/evidence_index.json)
- 封存目录说明：[`README`](../artifacts/milestones/lotusim_minimal_dynamics_20261028/README.md)
- 状态：**最小动力学闭环在上述限定范围内封存；C1–C6 并非全部完成。**

本文封存的是“当前合成双推进器 UUV 的最小动力学闭环证据”，不重新定义验收标准，不修改现有代码、`acceptance.json`、冻结记录或旧里程碑。后续若改变模型、控制器、场景、阈值或验收代码，应新建里程碑版本，不能覆盖本索引。

---

## 1. 封存范围

本里程碑覆盖从“模型/执行器”到“无流与恒流导航”的完整链路：

```text
v2 合成模型 / 双推进器分配
        ↓
M4–M6 数值检查（Python/xdyn）
        ↓
LOTUSim + xdyn 开环（方向、时间、限幅、回写、watchdog）
        ↓
PID 速度/正负俯仰阶跃 + 分配后抗饱和
        ↓
直线、深度、状态断流、过期状态、命令断流
        ↓
S1 单障碍恒流导航（F0/F1/F2）
        ↓
S2 缩比双障碍恒流导航（F0/F1/F2）
```

不包含：

- 全高 S2、S3、完整 45 次导航矩阵；
- 任意 6-DoF、剪切流、湍流、波浪、真实海况；
- 实艇辨识、实艇接入、完整通信/硬件故障；
- 论文终稿和答辩材料。

封存时的保护检查：

- 六份保护清单（`twin_*_protected_before.json`、`pid_*`、`motion_*`、`navigation_*`、`current_*`）共 413 条路径记录全部存在且 SHA-256 未变；
- 本次封存未修改模型、PID、LOS、分配器、恒流修正、验收标准或旧记录；
- 受保护文件的逐项哈希和检查结果记录在证据索引的 `protected_code` 字段。

---

## 2. 证据索引（人读版）

| ID | 类别 | 记录/文档 | 结果 |
|---|---|---|---|
| `bridge-minimal-xdyn` | 桥接/开环 | `artifacts/minimal_xdyn/validation_20261020_01` | 9/9 通过；最小 LOTUSim/xdyn 链路、方向/限幅/回写 |
| `model-v2-open-loop` | 模型/开环 | `artifacts/twin_axial_v2/validation_20261026_01` | 9/9 通过；Z=0、±15 N、回写、重复性通过 |
| `numerics-v2` | 数值 | `artifacts/twin_axial_v2_numerics/validation_20261026_01` | M4/M5/M6 通过；严格容差 |
| `numerics-current` | 数值/恒流 | `artifacts/twin_axial_current/numerics_20261027_final` | 24 项检查通过；F0/F1/F2 |
| `pid-steps` | PID | `artifacts/twin_axial_pid/validation_20261027_final` | 4/4；C1/C2 + 分配恢复 |
| `motion-fault` | 断流/运动 | `artifacts/twin_axial_motion/validation_20261027_final` | 6/6；直线/深度/三类故障 |
| `navigation-s1` | 导航 | `artifacts/twin_axial_current/navigation_20261027_final` | 13/13；F0/F1/F2，S1 单障碍 |
| `navigation-s2-scaled` | 导航 | `artifacts/twin_axial_current_double/navigation_20261028_final` | 13/13；F0/F1/F2，双障碍缩比 |
| `replay-s1` | 显示（非验收） | `artifacts/twin_axial_current_replays/verify_flow_retry` | 无界面回放 completed；箭头/参照球 |
| `replay-s2` | 显示（非验收） | `artifacts/twin_axial_current_double_replays/verify_headless_F2_P0` | 无界面回放 completed；两个挡板 |
| `docs` | 说明 | `docs/lotusim_*.md` 等 | 索引中逐文件哈希 |
| `code` | 代码 | 受保护清单 + 当前关键文件 | 索引中逐文件哈希 |

`evidence_index.json` 为每个 ID 保存路径、部分文件的 SHA-256、适用运行目录的 `acceptance.json` 哈希和关键指标。原始记录的完整哈希仍由各运行目录内的 `manifest.json` 负责；索引只封存“索引层”的关键文件，避免复制或改写原记录。

---

## 3. C1–C6 完成状态

论文范围标准（`dissertation_scope_baseline_v1.md`）当前状态：

| 标准 | 状态 | 证据 | 主要缺口 |
|---|---|---|---|
| **C1** 模型方程、参数、坐标与欠驱动关系 | **满足（限定范围）** | 最小 xdyn 桥接、v2 开环、M4–M6、恒流数值预检 | 合成模型、纵向平面；非实艇辨识、任意 6-DoF 未验证 |
| **C2** 可配置障碍、起终点、恒流 | **部分满足** | S1、S2 缩比、F0/F1/F2 | 全高 S2、S3、完整 45 次矩阵 |
| **C3** 实际动力学闭环 | **满足（限定范围）** | PID、motion/fault、S1/S2 导航；无 MOVE | 网关崩溃、网络/硬件故障、暂停时钟等完整故障矩阵 |
| **C4** 名义工况安全到达 | **部分满足** | S1/S2 缩比各 13/13 到达/净距/跟踪 | 全高 S2、S3、完整 N1–N6 矩阵 |
| **C5** 对照与适用边界 | **部分满足** | F0/F1/F2 的 S1/S2 对照 | 45 次矩阵、流速/阻尼敏感性、强流/剪切流 |
| **C6** 可复现系统与论文 | **部分满足** | Nix/lock、源码快照、manifest、测试、证据索引 | 论文终稿、图表、答辩包、独立机器复现 |

### 3.1 为什么 C1/C3 是“限定范围满足”

- 模型是明确标注的合成双推进器 UUV，不是实艇辨识模型；
- 验证集中在纵向 x–z 平面、无流和 F0/F1/F2 恒定均匀流；
- 真实求解器 xdyn 参与积分，LOTUSim 回写和位姿镜像通过；
- 所有导航运行都由动力学闭环产生，没有使用 MOVE 推进；
- 因而不把结论外推到任意 6-DoF、非均匀流或实艇。

### 3.2 关键结果摘要

| 项目 | 结果 |
|---|---|
| 最小开环桥 | 8+1 通过；160 步/次；重复差 0 |
| v2 数值 | M4/M5/M6 通过；Python/xdyn 最大位置差 9.32e-11 m |
| 恒流数值 | 24 项通过；F0/F1/F2 预检 |
| PID C1 速度 | 2.70 s 稳定；末 5 s MAE 0.001274 m/s；超调 0 |
| PID C2 俯仰 | ±20°：2.10 s；MAE 0.2291°；超调 0.9967° |
| 分配恢复 | 26 个饱和/回算步；速度 4.70 s、俯仰 2.80 s 恢复 |
| 直线/深度 | 15.4062 m；深度 20.9 s；末 5 s 深度 MAE 0.007156 m |
| 三类故障 | 状态 0.25 s、过期 0.05 s、命令 0.30 s 归零；持续零输入 |
| S1 导航 | 13/13；F1/F2 各 5/5；最小净距 1.62–1.74 m；最大 RMS 0.272 m |
| S2 缩比导航 | 13/13；F1/F2 各 5/5；最小净距 1.54–1.69 m；最大 RMS 0.338 m；最大路径误差 0.773 m |
| 当前代码测试 | 回放 15、恒流 19、双障碍场景 3（本次运行合计 37 passed） |

S1/S2 的详细逐次指标见对应 `suite.json` 和 `acceptance.json`；不要用上表的汇总替代逐次证据。

---

## 4. 剩余任务（按优先级）

### P0：完成论文范围所需场景

1. **全高 S2 与 S3**：当前 S2 是缩比有限挡板；全高 S2 在固定 0.4 m/s、60° 俯仰、±15 N 下会触碰包络。需要决定：
   - 调整速度/俯仰/执行器配置并重新冻结；或
   - 正式修改场景/包络定义并说明原因；
   - 不得用缩比 S2 冒充全高 S2。
2. **完整 45 次矩阵**：3 场景 × 3 水流 × 5 初态；同一控制器/规划/分配，失败不重试替换，先冻结 release。
3. **N1–N6 / R1–R2 全矩阵**：到达、超时、安全、路径跟踪、饱和、成功率、位姿回写、重复性。

### P1：C5 与故障边界

4. 更强流、流向组合、剪切/湍流不声称支持；先做 F1/F2 之外的受控流速和阻尼敏感性；
5. 完整 C3 故障矩阵：网关崩溃、网络中断、暂停时钟、硬件/执行器故障；当前只有控制消息边界三类故障；
6. 导航任务中的故障注入与终止/归零证据。

### P2：C6 与交付

7. 论文、图表、实验矩阵说明和答辩包；
8. 独立机器复现（Nix、manifest、索引校验）；
9. GUI 可见性在目标桌面确认；回放截图不作为验收。

---

## 5. 复现与校验

### 5.1 校验封存索引

```bash
python scripts/verify_lotusim_minimal_dynamics_freeze.py
```

期望输出 `"passed": true`，并显示每个证据条目的哈希、suite 状态和保护清单检查结果。该脚本只读，不启动 Gazebo/xdyn，不修改任何记录。

### 5.2 复核各阶段

```bash
# v2 数值
bash ros2/twin_axial_v2/run-nix.sh numerics-analyze \
  artifacts/twin_axial_v2_numerics/validation_20261026_01

# v2 开环
bash ros2/twin_axial_v2/run-nix.sh analyze \
  artifacts/twin_axial_v2/validation_20261026_01/pitch_positive

# PID
bash ros2/twin_axial_pid/run-nix.sh analyze \
  artifacts/twin_axial_pid/validation_20261027_final

# 直线/深度/故障
bash ros2/twin_axial_motion/run-nix.sh analyze \
  artifacts/twin_axial_motion/validation_20261027_final

# 恒流 S1 导航
bash ros2/twin_axial_current/run-nix.sh analyze \
  artifacts/twin_axial_current/navigation_20261027_final

# 双障碍缩比导航
bash ros2/twin_axial_current_double/run-nix.sh analyze \
  artifacts/twin_axial_current_double/navigation_20261028_final
```

### 5.3 只查看 Gazebo

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current_double/navigation_20261028_final
```

回放发送 MOVE，只用于检查模型/挡板/路径/水流箭头/参照球，不参与验收。

---

## 6. 封存规则

- 本索引和证据路径只读；后续实验使用新目录、新 `suite.json`。
- 不修改旧运行的 `acceptance.json`、`manifest.json`、`source/` 或冻结数据。
- 若模型、PID、LOS、恒流、场景或阈值改变，建立新里程碑 ID，并重跑受影响的完整证据集。
- 本里程碑的“完成”只指第 1 节范围和 C1/C3 的限定满足；C2/C4/C5/C6 仍有明确缺口。
