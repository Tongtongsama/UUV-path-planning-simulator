# UUV 转向决策可检验性预实验（2026-10-28）

## 已运行事实

### 审计与适用范围

仓库根目录没有 `AGENTS.md`。先读了 `README.md`、`config/navigation_v08.json`、基线控制器和合成车辆 YAML、`config/scenarios/planner_acceptance_v1.json`，以及下表代码。原有未提交改动未被本实验覆盖；本实验只新增 `experiments/uuv_turn_preexperiment/`，输出在已被 `.gitignore` 忽略的 `artifacts/uuv_turn_preexperiment_v3_20261028/`。未 push、merge 或改冻结配置。

| 链路 | 实际文件与接口 | 审计结论 |
| --- | --- | --- |
| 动力学 | `physics/uuv_3dof.py:Python3DOFBackend.step`、`simulation/vehicle_config.py` | 合成、非实船的 x–z 纵向垂直平面；状态为 `x,z,pitch,u,w,q`，ENU z 向上，SNAME w 向下，`z_dot = u sin(pitch) - w cos(pitch)`；默认固定步长 RK4。`r` 与 `tau_n` 均为非活动量，在本次 CSV 中逐步为 0。 |
| 控制 | `controller/cascaded_pid_3dof.py` 的 `step_with_limits`、`reset`、`last_diagnostics` | 同一基线 PID；每次任务开始重置一次，局部参考开始和结束时不重置。请求量在 `last_diagnostics.requested`，限幅输出在 `NavigationStep.commanded`，施加值在 `NavigationStep.applied`。 |
| 力与安全 | `navigation/navigator.py:Navigator.run`、`navigation/models.py` | `tau_x` 限 20 N，`tau_m` 限 8 N，未发现力/力矩速率限制、执行器状态或动态滞后。位置和角速度的到达阈值分别为 0.25 m、0.10 m/s、0.10 rad/s；默认没有独立的纵倾安全中止。 |
| 参考 | `trajectory/generator.py:generate_trajectory`、`trajectory/sampling.py:sample_reference` | 原参考沿折线匀速采样，折点使用下一段纵倾角，`q_ref=0`，折点处角度跳变。实验仅在控制器输入前的 `ReferenceAdapter._reference` 改局部参考，并保存实际送入 PID 的参考。生成器和全局路径不变。 |
| 几何 | `environment/world.py:segment_clearance`、`environment/obstacle.py:segment_clearance`、`planner/constraints.py` | Shapely 对每个已执行中心线段求连续障碍距离，并检查边界线段。只有规划用圆形代理半径 0.20 m，**没有真实船体长宽、船长 L、朝向相关外形或可验证的真实船体净间隙**。 |

因此这里可以直接输出 `q`、`tau_m` 和 `d_min`，也可以输出核心数据模型中的 `r`、`tau_n`；但不能把 `r` 说成此模型的转向角速度。实际转向量是 `q`，实际转向力矩是 `tau_m`。CSV 保存完整 12 维状态、有效参考、请求/限幅/施加控制、模式、逐步间隙与安全事件。`d_min` 定义为**中心到障碍距离减 0.30 m 安全包络**，故安全违规时为零或负；另存“中心距离”和“0.20 m 圆形代理外形净间隙”。生产状态 `COLLISION` 表示 0.30 m 安全包络被触及，本次基线尚未出现代理圆与墙体的物理重叠。无真实 L，`d_min/L=null`。

### 场景、协议与判据

沿既有 `planner_acceptance_v1` 的 `detour`：边界 `x=[0,10] m, z=[-10,0] m`；墙体 `x=[4.9,5.1] m, z=[-7,-3] m`；固定全局路径 `(1,-5) → (5,-7.5) → (9,-5)`，下侧几何通道从墙底到边界为 3 m。入段是首段，出口以最后目标 `(9,-5)` 为中心。两段参考纵倾角约 `-0.559` 和 `+0.559` rad，名义参考折点角跳变约 `1.117` rad。初始 `x=1,z=-5,pitch=0,u=w=q=0`；无水流、无随机扰动；参考速度 0.2 m/s，步长 0.05 s，统一超时 120 s。这里没有实船 `u_max`，不使用 `0.7u_max`。

外部 `success` 同时要求生产 `NavigationStatus.SUCCESS`、无安全中止，且完整圆形代理越过出口线 `x=8.8 m`（中心 `x>=9.0 m`），`|z+5|<=0.25 m`、`|pitch|<=0.2 rad`、`sqrt(u²+w²)<=0.1 m/s`、`|q|<=0.1 rad/s`。阈值在 `config.json` 预先固定。`d_min` 的安全阈值为 0；在代理外形净间隙的尺度上，安全阈值为 0.10 m。失败候选一律不参与成功候选的最优间隙排名，超时和原地转圈不算成功。

适配器的 identity 是 `alpha=1`，直接传递原参考。E1 是**参考形状强度**扫描，非独立力矩命令、非 `a_ref`：在全局首段弧长 `s=1.0…3.9 m` 添加向外（更深）的 `A sin²(pi(s-1.0)/2.9)` m 局部深度偏移，`A=2(1-alpha)` m，17 点，起始事件和程序长度 2.9 m 固定。由位置的一、二阶弧长导数计算切线纵倾、速度与 `q_ref`，与改动后位置运动学一致；偏移在两端的位置和切线连续，但 `q_ref` 端点仍可能跳变，原折点本身也未被圆滑化。E2 固定 `alpha=0.2`（`A=1.6 m`），将整段 `[1.0,3.9] m` 相对同一全局路径平移 `-0.8…+0.7 m` 共 21 点；负号为提前，程序长度始终 2.9 m。E2 改的是弧长触发位置，不是时间起点或单独的终止时间。E1/E2 控制器、限幅、全局路径、初始状态和物理后端相同。

### 结果

未包装基线运行一次、同一适配器的 identity 运行一次、identity 复跑一次；逐步记录与状态轨迹完全一致。单次约 0.6 s（基线）或 2.1 s（超时候选）。最终数据集共 72 次动力学运行：主批次 41 次（含基线复跑）、5×5 网格 25 次、半步长 3 次、E3 3 次；主批次墙钟 63.7 s，后续批次 54.2 s。之前对错误 E2 设计的诊断目录保留但不用于以下结论。

| 组别 | 成功 | 安全包络违规 | 超时 | 其他 |
| --- | ---: | ---: | ---: | ---: |
| E1，17 点 | 0 | 8 | 9 | 0 |
| E2，21 点 | 0 | 7 | 14 | 0 |
| 5×5 交互切片 | 0 | 10 | 15 | 0 |

基线在 19.75 s 中止，`d_min=-0.00699 m`，代理外形净间隙为 `+0.09301 m`，施加纵倾力矩峰值为 8 N·m 上限的 0.840 倍，未饱和。E1 最强点（`alpha=0, A=2 m`）的 `d_min=+0.15047 m`，但 **120 s 超时**：出口深度误差 `-0.572 m`、纵倾误差 `+0.326 rad`，累计绝对转角 `7.647 rad`，请求力矩峰值/上限 `1.202`，饱和步占比 `0.0071`，路径长 `11.10 m`。E1 `alpha=0.5` 也超时，`d_min=+0.00234 m`，累计绝对转角 `5.916 rad`。E2 固定强度下最高截断最小间隙为平移 `+0.49 m` 时的 `+0.46651 m`，仍是 120 s 超时；`+0.7 m` 时请求力矩峰值/上限达 `1.893`，饱和占比 `0.0092`。这些峰值都属于失败候选，不能作为安全通过改善。未出现同等控制输出的明显等效候选；请求幅值不同但限幅输出在少数步达到同一 8 N·m 边界。

E3 只选 E1 最强点，用既有持续终端跟踪与两个单列诊断：持续跟踪 120 s 超时；复用仓库 `TerminalCapturePolicy`，保持同一 PID 增益和力矩限幅，107.2 s 因捕获预算耗尽而 `CAPTURE_FAILED`；在参考结束后直接零施加控制的开放环自然演化于 49.65 s 越界。开放环结果不是固定低层控制器的局部参考方法。三者的终止前历史最小间隙均为 `+0.15047 m`，但各自失败，不参与最优排名。E3 没有推出一个有利的终止时刻。

对 identity、强参考、临界 `alpha=0.5` 做 `dt/2=0.025 s` 检查，状态类别均保持不变，`d_min` 分别为 `-0.00297,+0.15615,+0.00210 m`，对应原步长 `-0.00699,+0.15047,+0.00234 m`。近零临界点仍需更细时步或连续动力学碰撞分析。几何碰撞使用现有的**步内中心线段**连续检测；真实 RK4 曲线步间轨迹未获得形式保证。代表参考的连续采样弦均满足 0.30 m 包络和边界约束。最小间隙“位置”保存在 JSON 中为首次达到该值的执行线段起点，不宣称线段内部精确最近点。

运行了受影响的现有导航、轨迹、控制/物理、环境集成测试：`41 passed in 10.28s`。主脚本还断言 identity 与未包装结果完全相同、identity 复现一致、E1/E2 参数确实改变结果；后续脚本断言半步长结果类别、碰撞有符号间隙与出口成功判定。没有运行全仓测试，现有用户未提交的 ROS2/LOTUSim 工作也不属于本实验改动。

### 文件与复现

- 实现：`run.py`（`ReferenceAdapter._reference`、`run_case`、`make_plots`）、`followup.py`（网格、半步长、参考几何、E3）、`plot.py`；配置：`config.json`。
- 原始数据：`artifacts/uuv_turn_preexperiment_v3_20261028/` 中每次运行一个 CSV 与 JSON，另有 `run_summary.json`、`followup_summary.json`、`E1_scan.png`、`E2_scan.png`、`trajectory_pitch.png`、`key_poses.png`、`q_tau.png`。CSV 为逐步数据，JSON 有终态和每次汇总。`artifacts/` 按仓库规则忽略，不提交大日志。
- Windows PowerShell 复现：

```powershell
.\.venv\Scripts\python.exe -m experiments.uuv_turn_preexperiment.run --output artifacts/uuv_turn_preexperiment_repro
.\.venv\Scripts\python.exe -m experiments.uuv_turn_preexperiment.followup --output artifacts/uuv_turn_preexperiment_repro
.\.venv\Scripts\python.exe -m experiments.uuv_turn_preexperiment.plot --output artifacts/uuv_turn_preexperiment_repro
.\.venv\Scripts\python.exe -m pytest tests/integration/test_navigation.py tests/integration/test_trajectory_generation.py tests/integration/test_control_physics_integration.py tests/integration/test_environment_integration.py -q -p no:cacheprovider
```

运行环境：Python `.venv`，NumPy 2.2.6、Shapely 2.1.2、PyYAML 6.0.3、Matplotlib 3.10.9、pytest 8.4.2。配置种子 0，但此确定性批次没有随机采样。Git HEAD `6b68ea2343d6fcb286af0adad1e056ed4fe6e9f8`；运行前工作树已有 README、ROS2 回放、文档及其他未提交文件。本实验仅新增 `experiments/`；pytest 产生了一个未跟踪的 `pytest-of-DELL/pytest-7/` 临时目录，已恢复它清理掉的原跟踪测试夹具，但递归删除该新临时目录被自动审批拦截，未再改动它。`run_summary.json` 保存运行时 dirty 状态。

## 机制解释

局部参考确实改变了实际 `q`、请求/施加 `tau_m` 与安全包络间隙，说明仿真器具备检验“参考改动是否影响轨迹”的观测能力。E1 与 E2 均呈现从安全违规到避开墙体但无法完成出口的切换；5×5 交互网格也没有补出成功点。由于失败会早停，违规点的截断 `d_min` 不能同超时点的全程 `d_min` 统一排名。

强参考先让船体下潜更深，但同时带来数弧度的累计绝对转角、可超过 8 N·m 的请求峰值，随后原始折线参考在拐点产生约 1.117 rad 的姿态跳变。最终终端保持时的深度和纵倾偏差，以及 E3 捕获失败，说明“启动更早就能安全通过”尚未成立。饱和发生时间很短，不能把失败简单归为长期执行器饱和；参考折点、欠驱动耦合和终端交接都可能参与。启动位置对该场景的间隙有显著影响，但没有成功样本，不能把启动和终止的重要性作性能排序。

## 未验证假设与最小下一步

未验证真实船体几何和船长 L、执行器速度限制/动态、物理车辆参数、姿态安全包络、不同速度/初始偏置迁移，也未形成连续时间或形式安全保证。由于调参集没有成功候选，未挑选“获胜点”去做速度与偏置验证。

最小下一步是保留同一生产动力学、PID 和限幅，在**参考折点与终端交接**上构造一个位置、切线、`q_ref` 均连续的局部参考，并先在无障碍的同角度弯道验证其不会产生大角度原地旋转，再回到此障碍场景进行独立启动/终止切片与两个速度、少量初始偏置的留出验证。本轮结论严格是：**候选集未找到成功安全通过方案；不能称物理不可恢复，也不能称参考策略已改善任务成功率。**
