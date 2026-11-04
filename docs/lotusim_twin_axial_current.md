# 恒定洋流：坐标、动力学配对与单障碍导航

独立入口 `ros2/twin_axial_current/`。原 v2 模型、PID、分配后抗饱和、A*、路径捷径、LOS、无流导航入口及冻结记录均保留。新增内容是求解器内部的恒流水动力修正和独立验收工具；不修改原算法，不按水流分别调参。

## 启动步骤

1. 打开 PowerShell，进入 NixOS：

```powershell
wsl -d NixOS
```

2. 进入项目并给本次记录取唯一目录名：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
stamp=$(date +%Y%m%d_%H%M%S)
preflight="artifacts/twin_axial_current/numerics_$stamp"
```

3. 先运行水流方向、坐标、相对速度与 Python/xdyn 数值配对：

```bash
bash ros2/twin_axial_current/run-nix.sh numerics --output "$preflight"
```

应以 `"passed": true` 结束。程序直接调用真实 xdyn 求解器，此步没有 PID 或 Gazebo。导航命令必须指定通过检查的 `--preflight` 目录；检查失败不能跳过。第一次可能需要下载 Nix 依赖。

4. 运行一次向上流单障碍导航：

```bash
bash ros2/twin_axial_current/run-nix.sh run --flow F2 --case P0 \
  --preflight "$preflight" \
  --output "artifacts/twin_axial_current/F2_P0_$stamp"
```

可用 `--flow F0/F1/F2`、`--case P0/P1/P2/P3/P4`。默认无 GUI，自动启动 LOTUSim、xdyn、网关并清理。不要另开同一动力学试验；沿用端口 12356/12357、ROS domain 74，数值检查使用端口 12358。已有输出目录不会覆盖。中断或通信故障会归零并判未完成。

5. 完整验收：

```bash
bash ros2/twin_axial_current/run-nix.sh suite \
  --preflight "$preflight" \
  --output "artifacts/twin_axial_current/navigation_$stamp"
```

总计 13 次顺序运行：F0/P0 对照；F1、F2 各五组固定初态；各增加一次 P0 重复。需要十余分钟，性能慢的电脑可能更久。最终 `passed` 为总判定；`groups.F1`、`groups.F2` 分别给出成功率、安全与重复性。每种流至少 4/5 成功、P0 必须通过、所有试验安全、重复性通过；`all_runs_passed` 单独报告全部 13 次是否都通过。

6. 只读复核本轮已保存的数值检查或导航结果：

```bash
bash ros2/twin_axial_current/run-nix.sh analyze \
  artifacts/twin_axial_current/numerics_20261027_final

bash ros2/twin_axial_current/run-nix.sh analyze \
  artifacts/twin_axial_current/navigation_20261027_final
```

替换目录即可复核自己的试验，也可指定导航套件下的 `F2_P0` 单次目录。不会重新运行动力学或改写记录。不要把历史无流目录交给这个入口：历史数据仍使用原导航分析器。

## 坐标和速度含义

沿用项目的垂直平面：规划 x 向前、z 向上；NED 水流为 `(0, v_x, −v_z)`，Gazebo ENU 为 `(v_x, 0, v_z)`。位置仍有固定 z 偏置，速度不加偏置。

| 水流 | 规划 `(v_x,v_z)` / m/s | NED / m/s | 解释 |
|---|---|---|---|
| F0 | (0,0) | (0,0,0) | 无流对照 |
| F1 | (-0.05,0) | (0,-0.05,0) | 与名义前进方向相反 |
| F2 | (0,+0.05) | (0,0,-0.05) | 向上流 |

在俯仰 θ 为正抬头、roll=0、yaw=π/2 时：

```text
u_c = v_x cosθ + v_z sinθ
w_c = v_x sinθ − v_z cosθ
u_r = u − u_c
w_r = w − w_c
du_c/dt = −q w_c
dw_c/dt =  q u_c
```

`u,w` 是艇体坐标系中的对地速度，`u_r,w_r` 是相对水速。位置运动学使用对地速度；不能再给位置额外叠加一次流速。PID 沿用原对地速度反馈和原目标，不把相对水速偷偷替换为控制反馈。

实现采用完整姿态旋转 `v_c_body=Rᵀ v_c_NED`，`dv_c_body/dt=−ω×v_c_body`。导航日志中的水速是由固定环境配置和记录姿态推导的诊断量，不冒称传感器实测。

## 为什么需要独立修正力

已检查本地锁定版本的 xdyn 源码：`ConstantUWCurrentModel.cpp` 仅返回 NED 水平流，无法表示 F2；`DampingForceModel.cpp` 会把环境流转换到艇体后再扣除。原 v2 的附加质量 Coriolis 项则明确使用无流速度。

本入口保留原模型全部参数与力项，并追加一个 `constant_current_correction` maneuvering 力项，按每个 RK 子步的真实状态求值。内置 UW current 保持零，避免阻尼重复减流。该修正实现与原 Python 模型一致的方程：

```text
M ν_dot = τ − C_RB(ν)ν − C_A(ν_r)ν_r − D(ν_r)ν_r − g
          + M_A ν_c_dot
```

相对于无流 v2，追加的力为：

```text
Δτ = C_A(ν)ν − C_A(ν_r)ν_r
   + D(ν)ν − D(ν_r)ν_r + M_A ν_c_dot
```

F0 时修正为零。原质量、阻尼系数、恢复力矩、推进器及分配器完全保留。不能只替换阻尼而遗漏相对速度 Coriolis 和 `M_A ν_c_dot`，也不能用 ROS 上一帧状态近似这些积分器内部项。

这里是 LOTUSim 调用的 xdyn 水动力环境扩展，未新增 Gazebo 洋流渲染或流体场插件。所有水流均为空间均匀、时间恒定；不代表已支持剪切流、湍流、波浪或实测海况。

## 前置数值验收

- F0/F1/F2 各做 20 s 固定开环输入的 Python/xdyn 全过程配对，dt=0.05 和 0.025 s。
- 使用既定 M6：最大位置差 ≤0.05 m、俯仰差 ≤1°、u/w 差 ≤0.02 m/s。
- 步长对比：位置差 ≤0.02 m、俯仰差 ≤0.5°、速度差 ≤0.01 m/s。
- 静止、零推进输入：F1 应向负 x 漂移；F2 应向上漂移；F0 保持静止。
- 与水同速、水平零角速度：水相对速度为零，5 s 内速度和匀速位移偏差均 <1e-8；不能凭空产生阻力。
- ±30°、非零速度及俯仰角速度：1e-5 s 实际 xdyn 步所得导数，与独立 Python 方程最大分量差 <1e-4。
- 坐标往返误差 <1e-9；单元测试另外覆盖含 roll/yaw 的旋转和修正力向量式。

有流时不沿用“机械能必须递减”判据：水流可以对载具做功。M6 的实测验证范围仍是本垂直平面，不由此宣称任意六自由度海况都已验证。

## 导航验收保持原标准

S1 场景、规划留距、LOS、终点减速/制动及 PID 与上一轮相同，不按 F1/F2 分别调整。障碍仍从 LOTUSim 加载的同一份 SDF 读取。

| 项目 | 标准 |
|---|---|
| 到达 | 300 s 内进入目标半径 0.5 m，连续保持 2 s |
| 安全 | 全程相邻状态扫掠线段到障碍和边界中心净距 ≥0.9 m |
| 跟踪 | 全程有限路径折线距离 RMS ≤0.35 m、最大 ≤0.8 m |
| 饱和 | 分配 α<0.999 的控制步比例 ≤20% |
| 反馈/时间 | 0.05 s 连续时间、序号连续、无意外看门狗触发 |
| Gazebo 同步 | ≥95% 覆盖，位置 ≤0.01 m、姿态 ≤1° |
| 重复 | 同流 P0 的位置差 ≤0.02 m、到达时间差 ≤0.1 s |

安全是按艇体包络和几何扫掠评价，不依赖 xdyn 碰撞接触力。目标内保持 2 s 不等于长期悬停。

## 记录

沿用完整导航记录，并新增 `current.json`、`preflight_acceptance.json`、`preflight_manifest.json`。`plan.json` 记录本次流向；`model.yml` 包含实际求解器表达式。`navigation.jsonl` 的 `water` 字段保存世界流、艇体流、对地速度、相对水速及流速艇体系导数。状态、指令、抗饱和、真实施加输入和 Gazebo 位姿均逐步保存。

验收会重算 LOS/PID 和水流诊断，核对实际模型配置和哈希。原算法与冻结数据保护清单为 `artifacts/current_protected_before.json`。该阶段仅覆盖 S1/F1/F2，尚未完成全部 45 次正式场景矩阵。

## Gazebo 只读可视化

完整套件可以在 Gazebo 中只读回放，不启动 xdyn/PID/LOS：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current/navigation_20261027_final
```

显示内容包括 UUV、记录中的 `single_baffle_obstacle`、`plan.json` 的路径/起点/终点；F1/F2 下还有蓝色流向箭头和品红色随流参照球（按 `current_enu * elapsed` 做显示积分，不是 xdyn 刚体）。13 个用例用 `next`/`prev`/`case` 切换，箭头/参照球随流向更新，F0 时删除。说明见 [恒流单障碍导航的 Gazebo 可视化](lotusim_recorded_current_suite_replay.md)。

## 2026-10-27 实测结果

数值检查保存在 `artifacts/twin_axial_current/numerics_20261027_final`，24 项检查全部通过。F1/F2 的 0.05 s 配对最大位置差分别为 7.53e-11 m、9.04e-11 m；最大俯仰差分别约 3.43e-10°、3.30e-10°。静止漂移、同流匀速、倾斜姿态导数与坐标往返均通过。

导航记录保存在 `artifacts/twin_axial_current/navigation_20261027_final`，13 次全部通过。下表使用每种流五组固定初态中的最不利误差和净距：

| 工况 | 成功初态 | 到达时间 / s | 最小中心净距 / m | 最大 RMS / m | 最大路径误差 / m | 最大饱和比例 |
|---|---|---|---:|---:|---:|---:|
| F0 无流对照 P0 | 1/1 | 49.75 | 1.6225 | 0.2401 | 0.5997 | 0.604% |
| F1 逆向流 | 5/5 | 49.85–50.30 | 1.6469 | 0.2329 | 0.5540 | 0.603% |
| F2 向上流 | 5/5 | 49.50–50.00 | 1.7429 | 0.2717 | 0.7060 | 1.113% |

F1/P0 和 F2/P0 各重复一次，匹配状态位置差与到达时间差均为零；分别匹配 1005、998 个状态。所有运行 Gazebo 位姿覆盖率 100%，到达后实际输入归零。向上流增大了路径跟踪误差，结果仍满足既定阈值；不据此推断更强水流也会通过。

NixOS 在线验收与 Windows 只读重算均通过。13 次运行保存的 Python 源码快照均与最终代码一致；122 个既有文件哈希未改变，冻结数据校验通过。相关单元/回归测试 333 项通过。

开发目录 `numerics_20261027_01` 保留：求解器数据已生成，但初版报告中的 NumPy 布尔值导致 JSON 序列化失败。修正的是新报告代码；正式数值检查在新的 `numerics_20261027_final` 目录完整重跑，没有覆盖或替换失败记录。
