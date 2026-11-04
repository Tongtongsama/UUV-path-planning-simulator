# S2 恒流对照：F0、F1、F2

研究基线规定规划平面世界流速：F0=`(0,0)`、F1=`(-0.05,0)`、F2=`(0,+0.05)` m/s。三个工况使用相同双挡板 S2、LOS 参数、PID 增益、执行器分配、初态与验收阈值；唯一环境自变量是均匀恒流。真实闭环的数值状态仍由锁定版 xdyn 产生，LOTUSim/Gazebo 负责实体与显示。

## 1. 物理接入与数值核对

锁定的 lxdyn `363668b2` 中，`Body::calculate_state_derivatives` 在世界位置导数中另加 `current_in_ned`，而 `DampingForceModel` 对体轴速度扣除同一水流。两处同时启用会与本项目已批准的绝对对地速度方程不一致。因此本阶段没有启用其 `constant UW current` 环境模型；`current_model.py` 在 xdyn 的 maneuvering force 接口中按每个 RK4 子步的姿态计算相对水速、线性/二次阻尼、附加质量 Coriolis 与转动坐标中的 `M_A·dot(nu_c)`。位置运动学继续只由 xdyn 的绝对体轴速度决定。F0 模型逐字段保留原模型。

源代码依据：[锁定版 Body.cpp](https://github.com/naval-group/lxdyn/blob/363668b2b1f97e3177d2d617586fee6c4a727262/xdyn/core/Body.cpp)、[DampingForceModel.cpp](https://github.com/naval-group/lxdyn/blob/363668b2b1f97e3177d2d617586fee6c4a727262/xdyn/force_models/DampingForceModel.cpp)。

一次早期 F1/P0 试验 `s2_f1_p0_pilot_20261031` 触发保护性中止。逐步探针将其定位为 xdyn 表达式解析器对 `-q(t)` 等一元负号写法的错误求值；将同一项重排为 `A - q(t)*B` 后，首步受力方向正确。失败记录保留，不能作为正式 F1 对照。

正式 M6 配对入口，在 NixOS 中运行：

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash ros2/twin_axial_navigation/run-nix.sh current-numerics \
  --output "artifacts/twin_axial_navigation/current_m6_$(date +%Y%m%d_%H%M%S)"
```

现有结果 `artifacts/twin_axial_navigation/current_m6_20261031/report.json`：F1、F2 均通过 20 s 固定输入时间表的 Python/xdyn 配对。F1 最大位置差 `7.52e-11 m`，F2 为 `9.03e-11 m`，最大俯仰差均小于 `3.44e-10°`，`u/w` 差小于 `7.4e-13 m/s`，远低于 M6 的 `0.05 m`、`1°`、`0.02 m/s` 限值。输出保留实际模型、逐步求解器与参考轨迹、比较值及哈希清单。此项只验证受力和数值一致性，不证明导航成功。

## 2. 源码冻结与统一控制配置

正式 F1、F2 套件在冻结源码上运行：`artifacts/acceptance_20261031/source_freeze_20261031.json` 记录 88 个 Python 文件的 SHA-256，聚合值为

```
76dc0e47338cb4af74cc66757df76a861a20f42085d55c9fe22fbc43a8794eaa
```

两套冻结套件各自六次运行的 `source/` 快照都与该清单逐文件一致；`suite.json` 中 `source_snapshot_consistent=true`。只读复核见 `artifacts/acceptance_20261031/current_frozen_verification.json`（生成脚本 `verify_current_suites.py`）。

F0、F1、F2 的 P0 `controller.json` SHA-256 均为

```
BFF22C3E8F593714C3A7295D8455BCDCAF07C613DF21F376A337AC3BA9A5BFE0
```

确认没有按流速分别调整 PID、分配或俯仰限制。

较早的 F1 套件 `s2_f1_suite_20261031` 运行期间修改过 `acceptance.py`、`cli.py`、`gui_replay.py`；用当前分析器只读重算得到 `source_snapshot_consistent=false`，因此它只作为开发复验，不作为正式证据。其 P0 轨迹与冻结套件 P0 的位置差为 `0 m`，可用于交叉检查。

## 3. S2/P0 三流对照

运行单例：

```bash
bash ros2/twin_axial_navigation/run-nix.sh run --scene S2 --flow F1 --case P0 \
  --output "artifacts/twin_axial_navigation/s2_f1_p0_$(date +%Y%m%d_%H%M%S)"
bash ros2/twin_axial_navigation/run-nix.sh run --scene S2 --flow F2 --case P0 \
  --output "artifacts/twin_axial_navigation/s2_f2_p0_$(date +%Y%m%d_%H%M%S)"
```

| 工况 | 到达/中止 | 最小扫掠净距 | 与 F0 最大同时间位置差 |
| --- | --- | ---: | ---: |
| F0 | 140.95 s 到达，通过 | 1.2183 m | 0 m |
| F1 | 143.00 s 到达，通过 | 1.6092 m | 0.9174 m |
| F2 | 87.85 s 保护性中止，失败 | 0.8988 m | 0.6190 m（截止前） |

F2 在第二块挡板左侧 x≈12.101 m、规划 z≈−12.276 m 处越过 0.9 m 安全阈值；该失败不延长任务时间，也不重调 PID 以改写对照。P0 记录分别为 `s2_f0_release_20261031/P0`、`s2_f1_suite_frozen_20261031/P0`、`s2_f2_p0_pilot_20261031`。

静态对照图：`artifacts/twin_axial_navigation_visual/s2_current_p0_comparison_frozen_20261031/trajectory.png`（另有 PDF、`comparison.json`）。对照图直接使用冻结套件的 F1/P0；早期 `s2_f1_p0_pilot_02_20261031` 与其逐位置一致（0 m），两种记录给出同一曲线。

## 4. S2/F1 固定五初态套件（冻结源码）

`artifacts/twin_axial_navigation/s2_f1_suite_frozen_20261031/suite.json`：

| 初态 | 到达时间 (s) | 最小扫掠净距 (m) | 路径 RMS (m) | 最大路径误差 (m) |
| --- | ---: | ---: | ---: | ---: |
| P0 | 143.00 | 1.6092 | 0.2158 | 0.3575 |
| P1 | 142.60 | 1.6070 | 0.2152 | 0.3574 |
| P2 | 143.35 | 1.6114 | 0.2164 | 0.3575 |
| P3 | 142.70 | 1.6103 | 0.2165 | 0.3575 |
| P4 | 143.30 | 1.6081 | 0.2152 | 0.3574 |
| P0 重复 | 143.00 | 1.6092 | 0.2158 | 0.3575 |

五组初态 5/5 通过且全程安全；最大饱和比例 0.526%，低于 20% 限值。P0 重复的 2863 个匹配状态最大位置差为 `0 m`，到达时间差为 `0 s`。`suite.json` 的 `passed`、`all_runs_safe`、`source_snapshot_consistent`、`repeatability.passed` 均为 `true`。复跑入口：

```bash
bash ros2/twin_axial_navigation/run-nix.sh suite --scene S2 --flow F1 \
  --output "artifacts/twin_axial_navigation/s2_f1_suite_$(date +%Y%m%d_%H%M%S)"
```

该套件与此前 `s2_f1_suite_20261031` 的到达时间和净距一致；后者因源码快照不一致不作为正式证据。

## 5. S2/F2 固定五初态套件：失败分布

`artifacts/twin_axial_navigation/s2_f2_suite_frozen_20261031/suite.json`：五组初态 0/5 成功，六次运行（含 P0 重复）全部以 `Swept safety clearance violated` 中止，名义套件未通过。

| 初态 | 中止时间 (s) | 中止 x (m) | 中止 z (m) | 最小扫掠净距 (m) |
| --- | ---: | ---: | ---: | ---: |
| P0 | 87.85 | 12.1012 | −12.2761 | 0.8988 |
| P1 | 87.50 | 12.1032 | −12.2801 | 0.8968 |
| P2 | 88.25 | 12.1029 | −12.2805 | 0.8971 |
| P3 | 87.60 | 12.1039 | −12.2836 | 0.8961 |
| P4 | 88.15 | 12.1021 | −12.2767 | 0.8979 |
| P0 重复 | 87.85 | 12.1012 | −12.2761 | 0.8988 |

失败点取安全判定首次拒绝的状态，即最后一条导航更新之后的那一帧；其后的少量 transaction 是中止后的滑行，不计入失败位置。全部中止点集中在第二块挡板左侧同一段安全线上：x 跨度 `0.0027 m`，z 跨度 `0.0075 m`，时间跨度 `0.75 s`，净距都低于 0.9 m 阈值 1.2–3.9 mm。P0 重复与原 P0 的位置差为 `0 m`，1757 个匹配状态，失败原因与中止时间完全一致。

因此 F2 的失败不是单次偶然：在同一控制器和同一路径下，向上恒流把艇体推入第二块挡板的扫掠安全线。该记录作为失败边界证据保留；不通过降低安全阈值、延长任务或单独调 PID 来改写 `passed=false`。

## 6. GUI 与回放

恢复原艇体外形后的真实 GUI 试验 `artifacts/twin_axial_navigation/s2_f1_p0_gui_original_shape_20261031` 使用 S2/F1/P0，143.00 s 到达，最小净距 1.6092 m，验收 `passed=true`；真实试验中不再显示青色跟踪球，艇体、艇首和双推进器保持原外形与部件颜色。该检查与只读回放分离：跟踪球只存在于回放。

GUI 双球对照（NixOS 终端）：

```bash
bash ros2/twin_axial_navigation/run-nix.sh replay \
  artifacts/twin_axial_navigation/s2_f0_release_20261031/P0 \
  --reference artifacts/twin_axial_navigation/s2_f2_p0_pilot_20261031 \
  --include-failed-reference \
  --output "artifacts/twin_axial_navigation_visual/s2_f0_f2_gui_$(date +%Y%m%d_%H%M%S)"
```

青球是 F0，橙球是 F2；橙球在原失败点停止，之后保持在那里。`p` 暂停/继续、`r` 重播、`0.25`/`0.5`/`1`/`2`/`4`/`8` 改倍速、`q` 退出，每条指令后按 Enter。GUI 回放仅显示原始轨迹，不能将失败记录变成成功。新回放自动使用独立 ROS domain，避免与已有回放互相响应。

## 7. 当前范围与下一步

已建立：S2/F0 与 S2/F1 的五初态套件通过；S2/F2 的五初态在同一安全边界失败且可重复；三种流速使用同一控制器配置；M6 的 Python/xdyn 受力配对通过。较早的 F1 套件只作为开发复验，正式结论以冻结套件为准。

尚未建立：F2 名义工况的成功导航；流感知引导或路径偏移；S1、S3、其他流速/流向及完整 45 次矩阵；更强流、剪切流或参数不确定性的适用边界。后续若要改善 F2，应单独研究流感知引导/路径或失败边界，并保持 F0/F1/F2 同一控制配置再做新的成组比较。

本机全量 Python 回归为 1004 项通过，JUnit 记录在 `artifacts/acceptance_20261031/full_regression_current_frozen.xml`。
