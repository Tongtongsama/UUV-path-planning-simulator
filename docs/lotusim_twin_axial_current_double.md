# 双障碍恒流导航（S2 缩比场景）

日期：2026-10-28。独立入口 `ros2/twin_axial_current_double/`。复用**未修改**的 `uuv_twin_axial_v2` 模型、PID 适配器、双推进器分配、S1 的 A*/LOS 和 `constant_current_correction`，只新增双障碍场景、运行入口和验收封装。现有 S1 记录、旧模型、冻结数据不覆盖。

相关：[恒流 S1](lotusim_twin_axial_current.md)、[Gazebo 回放](lotusim_recorded_current_suite_replay.md)、[阶段总览](lotusim_integration_and_stage_report_20261027.md)。

## 1. 场景与“缩比”的原因

研究范围基线中的 S2 全高挡板是：

```text
障碍 A: x [6,7],  z [-20,-7]
障碍 B: x [13,14], z [-13,0]
边界:   x [0,20], z [-20,0]
起点:   (2,-10)   目标: (18,-10)
```

直接用当前固定配置（0.4 m/s、俯仰限幅 60°、±15 N）运行时，全高 S2 要求车辆先越过 A 顶部（z>-7），再在 B 前降到 z<-13，垂直反转约 10 m。实际试跑会触发既有的保护/包络，不能作为“通过”的结果。

因此本入口使用**同样“先越过下方障碍、再从上方障碍下方通过”的缩比双障碍**：

| 障碍 | 规划 x 范围 | 规划 z 范围 | Gazebo ENU 中心 | 尺寸 (x,y,z) |
|---|---|---|---|---|
| `baffle_lower` | 6–7 | -13 – -10 | (6.5, 0, -31.5) | 1 × 4 × 3 |
| `baffle_upper` | 13–14 | -10 – -7 | (13.5, 0, -28.5) | 1 × 4 × 3 |

路径需要从 A 上方通过，再从 B 下方通过；要求的垂直反转约 5 m，而不是全高版本的约 10 m。边界、起点、目标、规划留距、LOS、PID、推进器限幅、恒流配置和验收阈值都保持不变。

`plan.json` 的场景 ID 是 `double_baffle_feasible_v1`。这是**缩比双障碍**，不是“原始全高 S2 已通过”。如果必须使用全高 S2，需要重新评估速度/俯仰/执行器配置或明确改变验收包络；不能用缩比结果冒充全高结果。

## 2. 复用了什么

- 模型：`ros2/twin_axial_v2/model.py`，未改；
- PID 与分配后抗饱和：`ros2/twin_axial_pid/control.py`，未改；
- 双推进器分配：`ros2/twin_axial_v2/allocation.py`，未改；
- LOS/A*/捷径/轨迹：`ros2/twin_axial_navigation/mission.py` 中的同一套类；
- 恒流：`ros2/twin_axial_current/model.py` 的 `constant_current_correction`；
- 数值预检：直接复用 S1 的 `numerics_20261027_final`，因为模型/水流配置相同；
- 新包只负责 S2 的世界/路径/验收包装。

S2 规划路径（P0、F0）为：

```text
规划 (2,-10) → (6.5,-8.0) → (9.5,-9.5) → (13,-12.0) → (17.5,-10.5) → (18,-10)
```

路径由现有 `AStarPlanner + generate_trajectory(shortcut) + simplify_path` 生成并通过 `validate_path`；没有按场景手写路径，也没有为通过而调 PID。

## 3. 运行

先确认 S1 的数值预检存在（本任务不重复跑）：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
ls artifacts/twin_axial_current/numerics_20261027_final/acceptance.json
```

### 整套 13 次

```bash
bash ros2/twin_axial_current_double/run-nix.sh suite \
  --preflight artifacts/twin_axial_current/numerics_20261027_final \
  --output "artifacts/twin_axial_current_double/navigation_$(date +%Y%m%d_%H%M%S)"
```

共 13 次：F0/P0；F1、F2 各 P0–P4；F1/P0、F2/P0 各一次重复。默认无 GUI；程序自行启动/清理 xdyn、网关和 LOTUSim。端口 12356/12357、ROS domain 74，不能与另一份同端口试验并行；输出目录必须全新。

### 单次

```bash
bash ros2/twin_axial_current_double/run-nix.sh run \
  --flow F2 --case P0 \
  --preflight artifacts/twin_axial_current/numerics_20261027_final \
  --output "artifacts/twin_axial_current_double/F2_P0_$(date +%Y%m%d_%H%M%S)"
```

`--flow` 为 F0/F1/F2，`--case` 为 P0–P4。

### 只读复核

```bash
bash ros2/twin_axial_current_double/run-nix.sh analyze \
  artifacts/twin_axial_current_double/navigation_20261028_final
```

也可传入单个 `F2_P0` 子目录。`analyze` 不启动动力学，只读记录、核对 manifest、重算 LOS/PID/分配和 S2 安全/跟踪指标。

## 4. 验收标准

与 S1/无流导航保持同一组阈值，不按场景或流向调整：

| 项目 | 标准 |
|---|---|
| 到达 | 300 s 内进入目标半径 0.5 m，连续保持 2 s |
| 安全 | 全程相邻状态扫掠线段到两个障碍和边界中心净距 ≥0.9 m |
| 跟踪 | 全程有限路径折线距离 RMS ≤0.35 m，最大 ≤0.8 m |
| 饱和 | 分配 `alpha<0.999` 的控制步比例 ≤20% |
| 反馈/时间 | 0.05 s 连续、序号连续、无意外 watchdog |
| Gazebo 同步 | 覆盖率 ≥95%，位置 ≤0.01 m、姿态 ≤1° |
| 重复 | 同流 P0 位置差 ≤0.02 m、到达时间差 ≤0.1 s |

套件判定：F1、F2 各至少 4/5 初态成功，P0 必须成功，所有运行安全，重复性通过；`all_runs_passed` 另报 13 次是否全部通过。

## 5. 实测结果

完整套件：`artifacts/twin_axial_current_double/navigation_20261028_final`。**13 次全部通过**，`suite.json` 为 `passed=true`、`all_runs_passed=true`：

- F0/P0 1/1；
- F1 P0–P4 5/5；
- F2 P0–P4 5/5；
- F1/P0、F2/P0 各重复一次，匹配 1009、996 个状态，位置差和到达时间差均为 0；
- 所有运行 Gazebo 位姿覆盖率 100%。

| 工况 | 成功初态 | 到达时间 / s | 最小中心净距 / m | 最大 RMS / m | 最大路径误差 / m | 最大饱和比例 |
|---|---:|---:|---:|---:|---:|---:|
| F0 无流 P0 | 1/1 | 49.75 | 1.5373 | 0.3254 | 0.6674 | 0.805% |
| F1 逆向流 P0–P4 | 5/5 | 50.10–50.55 | 1.5378 | 0.3030 | 0.6209 | 0.500% |
| F2 向上流 P0–P4 | 5/5 | 49.45–49.90 | 1.6226 | 0.3383 | 0.7729 | 0.503% |

F2 的路径最大误差 0.7729 m 最接近 0.8 m 阈值，但仍通过；不能据此推断更强流或更紧障碍也会通过。

Windows 只读复核 `analyze` 也返回 `passed=true`；`--preflight` 指向 S1 的 `numerics_20261027_final`，因为模型与恒流配置相同。新增 S2 场景测试 3 项通过；既有恒流测试 19 项、回放测试 15 项也通过。

## 6. Gazebo 可视化

新包记录仍是标准 `gazebo_poses.jsonl` + `worlds/minimal.world` + `plan.json` + `current.json`，现有回放入口可以直接读取：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current_double/navigation_20261028_final
```

显示内容：UUV、两个记录中的挡板、计划路径/起点/终点、蓝色流向箭头和品红随流参照球；13 个用例用 `next`/`prev`/`case` 切换。回放仍是 MOVE 显示，不是动力学验收。

实测：`--validate-only` 读取 13 例全部通过；`--prepare-only` 生成的 world 包含 `baffle_lower`、`baffle_upper`、`recorded_mission_overlay`，没有 collision；无界面 `F2_P0` 回放为 187 条 UUV MOVE、266 条观测，`status=completed`；`gazebo.log` 创建了两个挡板、路径叠加、`flow_arrow` 和 `current_drifter`。输出在 `artifacts/twin_axial_current_double_replays/verify_headless_F2_P0/`。

## 7. 边界

- 本场景是双障碍缩比版，不是全高 S2；见第 1 节。
- 只覆盖 F0/F1/F2、P0–P4 和两次重复，不是完整 45 次矩阵。
- 不覆盖任意流场、剪切流、湍流、波浪或真实海况。
- 回放、Gazebo 画面不能替代 `acceptance.json`。
- 所有旧 S1 记录、模型、PID、LOS、冻结数据保持不变。
