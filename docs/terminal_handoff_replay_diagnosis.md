# Terminal handoff 可达性：独立 replay 诊断

状态：Step 0 已提交 evaluated baseline；Step 1 两个最小诊断完成。
未修改生产 capture、guidance、PID、Physics 或安全条件。未将结果解释为 LOS 已达到 12/16。

## 1. evaluated baseline 已进入 Git

本地提交：`66a973c49cbf62faacab08ba3985f918dad7bfa3`。
提交说明：Record evaluated LOS and timed navigation baselines with matched evidence reports。
包含原先未提交的 Phase A/B、post-freeze startup、配置、测试、审计与 matched 报告。
不是 superiority release，未推送、未创建 release tag，artifacts 没有进入提交。
原候选快照及 matched 64 次运行证据保留，未覆盖或重新执行整条路径。

## 2. Replay 的真实性与边界

输入为 `artifacts/guidance_matched_16_v1/los/` 中保存的 detour 和垂向流结果，
以及相应 common 配置。先验证原 evidence manifest。

- detour：取 capture 拒绝时最后的真实状态（t=30.55 s）。
- current：取第一条 TERMINAL_CAPTURE 控制之前的状态（t=18.10 s）。
- 用已保存状态/target/limits 重放 **Controller 调用**，逐步核对输出和积分记忆；
  不重新规划、不执行入口之前的 Physics，也不以零积分冒充原控制器。
- 两个 original replay 的全部后缀状态与已保存结果 **逐项精确相等**。
- 每个分支再重复一次并比较全部诊断结果。四个分支均确定性一致。
- 后续 Physics 使用原 dt、限幅、连续线段碰撞、侵蚀边界、目标容差和保持时间。
  不增加原任务时限或原 capture 60 s 预算。

入口控制器积分：detour surge=2.53855、pitch=0.17033；current surge=1.31980、pitch=-0.10638。
这些记忆没有在 handoff 时重置。

## 3. detour_handoff_replay

原入口：位置误差 0.2133 m，平移速度 0.5974 m/s，q=0.5358 rad/s。
原 capture guard=0.35 rad/s，直接入口拒绝被完整复现。

另做一个 **仅 validation 的预捕获制动 probe**，不是新的生产 policy：

- 固定入口实际 pitch，q_ref=0，不立即跳到末段方向；
- 使用原 brake surge target（反向目标幅值最多 0.15 m/s）和原 inner PID；
- 预算 5 s，距入口最大允许位移 0.5 m；数值均在运行前写入配置，没有结果后调参；
- 诊断阶段实际 pitch 保持原 70° guard，角速度不允许增大到入口幅值以上；
  **这不是放宽 capture 的 0.35 guard**：capture 尚未激活；
- 只有降到原 brake_speed=0.03 m/s、brake_pitch_rate=0.03 rad/s 才尝试交接；
- 连续实际运动安全检查始终开启，保留第一个采样阈值违规状态作为证据。

结果：1.25 s 后入口位移约 0.501 m，返回 STABILIZATION_DISPLACEMENT_FAILED，
未进入 capture。此时 q=-0.0337 rad/s，平移速度仍 0.2542 m/s，目标误差 0.3484 m。
角速度能降低，但车辆在此期间继续前进并越过目标。

解释：**仅在这个过晚入口增加等待/制动不能满足本次预设空间预算。**
这不是所有预捕获策略均不可行的证明；5 s/0.5 m 是显式 probe 条件，不是已识别的车辆能力。
下一项应研究更早的接近/制动与 handoff 状态集合，不能只解除 q guard。

## 4. current_capture_brake_replay：速度语义与可达性

Physics 实现为 `eta_dot = J(theta) @ nu`，`nu_relative = nu - current_body`。
因此：

- `Twist.u/w` 是 **body 坐标中的 absolute/对地速度**，不是相对水速度；
- `hypot(u,w)` 等于 world 对地速度模长（平移映射保持模长）；
- 相对水速度必须另外计算 `norm([u,w] - current_body)`。

原 replay：全部 1200 步为 BRAKE，60 s 后预算耗尽，误差 3.0567 m。
末态对地速度约 0.0500001 m/s，相对水速度约 0.0000919 m/s。
所以“相对水已经几乎静止”不能等同于“对地已经停稳”。

### 固定水平姿态的解析检查

在 theta=u=q=0、world current=[0,+0.05] 下，body current=[0,-0.05]。
无 heave actuator，当前质量/阻尼给出静止状态的：

```
w_dot = -(6*0.05 + 1.5*0.05^2) / 28
      = -0.0108482143 m/s^2
```

`u=0,w=-0.05,q=0,theta=0` 是速度平衡：nu_dot=0，但 z_dot=+0.05 m/s。
对地速度 0.05 大于 BRAKE 的 0.03 阈值。现有 BRAKE 持续保持末段水平姿态，
因此该固定姿态下的漂流平衡无法满足其退出条件。

这不是“所有姿态/动作下均不可能捕获”的证明，也不是仅修改速度字段就能修复：
直接改成 water-relative 判断会把持续漂移误当成停稳，并改变任务语义。

### 单因素对照

从同一入口状态和同一 PID 记忆开始，仅把**后续**海流设为零；其余均不变。
17.30 s 后 SUCCESS，误差 0.22945 m，速度 0.02820 m/s，沿用原保持条件。
经历 BRAKE/ALIGN/MOVE/SETTLE，未增加预算或放宽阈值。
这是 counterfactual，不是“修复了有流任务”，也不回填原 matched 的成功统计。

## 5. Planner 提案需修正的合同细节

现有 startup 检查 `abs(first_heading) <= 70°`（绝对 nose-up pitch），
并检查初始实际 pitch/rate、漂移与预算。它不是只检查
`abs(wrap(first_heading - initial_pitch)) <= 70°`。
例如起始和首段均为 80°，相对差为零仍不满足现有绝对 envelope。

未来 initial-heading validation 必须复用实际 startup 可接受条件；若再加相对转角约束，
应作为另一个有依据的条件，而不是替换绝对 guard。单次 heading 检查也不能保证整个
启动 transient 可执行。未来膨胀还应保留已有 vehicle_radius + safety_margin，
再显式加入独立测得的 execution margin，避免遗漏或重复计算已有安全项。

## 6. 交付与后续顺序

脚本：`validation/terminal_handoff_replays.py`。
配置：`config/terminal_replay_diagnostics.json`。
原始输出：`artifacts/terminal_handoff_replays_v1/`（四个 replay、summary、manifest）。

```powershell
.\.venv\Scripts\python.exe -m validation.terminal_handoff_replays --output artifacts/terminal_handoff_replays_v1
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider --junitxml=artifacts/terminal_handoff_replays_tests_v1.xml
```

复跑使用新目录。新增 6 项测试覆盖 frame/relative speed、解析漂流平衡、PID 记忆恢复与
损坏证据拒绝、原 capture rate guard。不改生产模型。
本轮全仓回归：**763 passed**。原 matched evidence 与新 replay manifest 均已校验，
生产运行时源码与冻结候选的 SHA-256 一致。

Step 2（转弯执行包络）和 Step 3（execution-aware Planner）尚未实现。
先将这两个 terminal 问题形成独立设计合同，再测量转弯能力；不要直接把任意膨胀或
转角惩罚当作已验证可执行性。Geometric A*、timed 和 LOS 三个基线继续保留。
ROS 2 仍为第一扩展目标，不替代当前诊断。
