# uuv_twin_axial_v2：无流 PID 阶跃验收

日期：2026-10-27。独立入口 `ros2/twin_axial_pid/`。沿用原样的 `uuv_twin_axial_v2` 模型、网关和双推进器分配器；原 `controller/` 算法、旧控制器 YAML、模型和冻结数据不变。

本轮只验收前进速度和正负俯仰内环，以及分配饱和后的恢复。没有 LOS、规划路径、深度外环、洋流或正式导航实验。

## Windows / NixOS 启动

1. 打开 PowerShell：

```powershell
wsl -d NixOS
```

2. 进入项目：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
```

3. 一次完成四个用例：

```bash
bash ros2/twin_axial_pid/run-nix.sh suite \
  --output "artifacts/twin_axial_pid/suite_$(date +%Y%m%d_%H%M%S)"
```

不需要手动启动 LOTUSim、xdyn 或 UI。程序自己启动三个子进程，默认无 GUI，以避免图形卡顿影响控制试验。预计需要数分钟；等最后打印 suite JSON。每次用全新目录，不能覆盖旧记录。

沿用端口 12356/12357 和 ROS domain 74，不要与 v1/v2 开环或另一份 PID 试验同时运行。端口已占用时入口直接报错，不结束别人的进程。Ctrl+C 会保留中断记录，但中断试验不能作为通过证据。

4. 单独重跑一个用例（将 case 改成下表名称）：

```bash
bash ros2/twin_axial_pid/run-nix.sh run --case pitch_positive \
  --output "artifacts/twin_axial_pid/pitch_positive_$(date +%Y%m%d_%H%M%S)"
```

5. 只读复核完整套件或单次记录：

```bash
bash ros2/twin_axial_pid/run-nix.sh analyze \
  artifacts/twin_axial_pid/validation_20261027_final
```

查看自己的结果时，将最后一行换成刚才实际输出目录。`analyze` 校验记录哈希并重算指标，不运行动力学，不修改原记录。返回非零或任何 `passed: false` 都需要查看具体失败项，不能只检查窗口是否显示。

## 固定用例与验收标准

| case | 目标 | 指标 |
|---|---|---|
| speed_step | 0–2 s 静止；2–32 s u=0.4 m/s，pitch=0 | C1：15 s 内进入 ±0.04 m/s 并保持；末 5 s MAE≤0.02 m/s；超调≤0.08 m/s |
| pitch_positive | 2 s 同时给 u=0.4 m/s、pitch=+20°，持续 30 s | C2：15 s 内进入 ±3° 并保持；末 5 s MAE≤2°；超调≤5°；同时检查 C1 速度指标 |
| pitch_negative | 同上，pitch=-20° | 同上 |
| allocation_recovery | 2–10 s 给 u=0.6 m/s、pitch=60°；10–40 s 改为 u=0.4 m/s、pitch=0 | 必须真正触发联合饱和和积分修正；恢复目标后 15 s 内速度/俯仰进入上述误差带并保持，末 5 s MAE 同上，末 5 s 分配退出饱和 |

最后一个是额外抗饱和开发试验，不替代正式 C1/C2。它不套用首次阶跃的超调指标：恢复到零俯仰时初值接近 60°，报告中的最大偏离不能解释为从零目标产生了 60°超调。

所有指标按仿真时间，稳定时间要求进入误差带后一直保持到测量终点，不能只取第一次穿越。三项正式阶跃全程不裁剪启动过程。测量结束后额外约 0.15 s 发送零输入，并确认实际求解器收到零输入；这一小段不计入跟踪指标。

每次还检查完整记录、时间连续、单台 ±15 N 限幅、Z 执行器输入为零、平面/速度/俯仰包络、无意外 watchdog 中断、Gazebo 位姿回写覆盖率≥95%、正常退出，以及 PID 请求到实际施加推力的对应关系。

`suite.json` 应为 `passed: true` 且四个 `exit_code` 均为 0。每个子目录的 `acceptance.json` 所有 checks 均须为 true。

## 复用 PID 与分配后抗饱和

实际闭环为：xdyn 状态 → LOTUSim/网关反馈 → 原 PID 的 `step_guidance` 内环 → X/M 请求 → 原双推进器联合分配 → xdyn 积分 → 下一帧状态。这里没有 Python 预测动力学代替真实求解器。

适配器继承原 `CascadedPID3DOFController`，使用它的速度 PI、俯仰 PID、角度处理及正弦恢复力前馈。只在新适配器中管理自己的积分记忆。位置/深度外环关闭，目标直接传入 `BodyGuidance3DOF`。

本阶段独立固定参数：速度 Kp=25、Ki=3；俯仰 Kp=20、Ki=1、角速度 Kd=22；恢复前馈系数=6；积分限幅分别 ±4、±1。它们是本合成模型的独立候选参数，不覆盖旧 YAML，也不宣称为实艇辨识/整定结果。四个用例共用一套参数。

旧 PID 的 X/M 独立限幅在此适配器中设为大内部界限 1e6，仅用于获得原始请求；真实物理限幅始终由双推进器分配器实施。不得将 1e6 写成执行器能力。

每步先按原 PID 更新候选积分，形成原始 X/M 请求。分配器返回 `alpha`、上下推力和可实现 X/M 后，使用：

```text
Iu_next     = clip(Iu_candidate     + dt*(X_alloc-X_request)/(Ki_u*Tt), ±4)
Itheta_next = clip(Itheta_candidate + dt*(M_alloc-M_request)/(Ki_theta*Tt), ±1)
Tt = 1 s, dt = 0.05 s
```

修正影响下一控制步。两个轴均使用联合分配后的力/力矩，不使用旧独立裁剪值。网关收到原始 X/M 后独立重复同一分配，记录实际施加值；验收核对它与 PID 记录一致。上述抗饱和针对当前理想推力模型，不涵盖未建模电机滞后或执行器故障。

状态无效、跳帧、仿真时间不连续或超过 0.25 s 墙钟无反馈时，入口停止控制、清空积分并尝试发零，随后结束试验；独立网关仍保留 0.25 s 仿真时间/1 s 墙钟的命令过期归零。正常阶跃验证不等于完成新的闭环 C3 故障注入验收，C3 仍需在后续专门测试。

## 记录与复核

- `controller.json`：独立参数及抗饱和跟踪时间。
- `commands.jsonl`：参考值、真实反馈序号、原始 X/M、分配后推力、alpha、候选积分、回算项及修正后积分。
- `transactions.jsonl`：完整 xdyn 状态、实际施加推力与广义力、watchdog 和仿真时间。
- `gazebo_poses.jsonl`：实际 LOTUSim 位姿反馈。
- `source/`、`flake.lock`、`model.yml`、日志及 `manifest.json`：复现依据与哈希。
- `acceptance.json`：固定阈值下的指标和判断；`suite.json` 为套件汇总。

只读验收重算每一步 PID 和抗饱和，并核对已记录的实际输入。跨 Windows/NixOS 的浮点重算允许绝对/相对 1e-12 舍入差，不放宽 C1/C2 物理阈值。

在 Gazebo 中只读回放本套件（不启动 xdyn/PID，按用例切换）：

```bash
bash ros2/recorded_replay/run-nix.sh artifacts/twin_axial_pid/validation_20261027_final
```

说明见 [双推进器 PID 验证套件的离线可视化](lotusim_recorded_pid_suite_replay.md)。

本阶段后续工作应是无障碍直线/深度变化及闭环断流注入；通过后再接 LOS 和规划路径。含流模型验证仍独立待办。

## 本次实测结果（2026-10-27）

最终套件：`artifacts/twin_axial_pid/validation_20261027_final`。四项均通过，退出码均为 0；Windows 只读复核也通过，记录哈希与当前源码快照匹配。动力学由真实 LOTUSim/xdyn 产生。

| 用例 | 进入误差带并保持 | 末 5 s 平均绝对误差 | 超调 |
|---|---|---|---|
| 速度 0→0.4 m/s | 2.70 s | 0.001274 m/s | 0 |
| 俯仰 0→+20° | 2.10 s | 0.2291° | 0.9967° |
| 俯仰 0→-20° | 2.10 s | 0.2291° | 0.9967° |

两项俯仰试验同时保持前进目标；速度稳定时间 3.10 s、末 5 s MAE 约 0.001560 m/s、无速度超调，也通过 C1 数值要求。两项各有 3 个控制步触发联合饱和及回算。

额外饱和恢复试验触发 26 个饱和/回算控制步；10 s 降低目标后，速度 4.70 s、俯仰 2.80 s 进入并保持误差带，末 5 s MAE 分别 0.002512 m/s 和 0.1301°，分配已退出饱和。没有与禁用抗饱和的完整动力学对照，因此这里只证明实现执行及恢复通过，不声称改善百分比。

三项标准试验各记录 643 个动力学步，恢复试验 803 步；Gazebo 位姿观测覆盖率均为 100%。相关测试 `169 passed`。61 个原模型/算法/配置受保护文件哈希未变，冻结数据校验通过。

开发记录 `development_20261027_01` 也保留。初版的 Windows 只读重算因约 4.44e-16 浮点差异报告不匹配；修复的是记录比较的舍入容差，未调控制增益或物理验收阈值。最终套件为修复后完整重跑结果。
