# 受控启动验证与角点局部减速

日期：2026-09-17。可选启动候选已实现；局部减速仍是 validation 中的实验生成器。
默认配置、PID gains、Physics、执行限制、目标容差均未改变；没有加入路径平滑。

## 受控启动合同

显式使用 `Navigator(..., startup_policy=ControlledStartupPolicy())`，默认 None。
要求控制器支持 `GuidanceController3DOF`，复用正式内环、前馈和限幅/anti-windup，
不伪造当前位置参考。没有修改初始姿态、瞬移或释放时重置积分。

BRAKE → ALIGN → RELEASED：

- 速度 ≤0.03 m/s 且 |q|≤0.02 rad/s 后才允许改变航向指令。
- BRAKE/ALIGN 请求与实际 surge 相反、幅度最多 0.15 m/s 的目标用于制动。
  首段航向限制 ±60°，姿态参考以最多 0.15 rad/s 从实际初始 pitch 向首段方向变化。
- 实际 pitch 误差 ≤0.03 rad、速度 ≤0.03 m/s、|q|≤0.02 rad/s 连续保持 0.5 s 后释放。
- 预算 20 s，漂移上限 0.10 m；实际 |pitch|>70° 或 |q|>0.35 rad/s 拒绝继续，
  返回 STARTUP_FAILED。实际运动碰撞/越界始终通过原安全流程检查。
- 空间或预算不足保留失败，不保证在任意流场下原地转向/定点保持。

### 参考时钟

物理时间继续推进并计入原 120 s 任务预算；只在启动期间冻结路径参考时钟。
释放后路径时钟按固定步长推进，不追赶暂停时间。终点捕获入口也加上启动延迟。

原生成轨迹保留在 `result.trajectory`；每步 `reference_time` 是轨迹查询时间。
`reference_history` 保存对应查询值并使用物理时间戳；实际启动指令单独存于
`effective_guidance`，阶段为 `startup_phase`。不能将暂停后的跟踪误差称为
相对原绝对时间表的误差改善。`reference_duration` 不含启动；总耗时包含启动和捕获；
`startup_duration` 单列，`settling_time_after_reference` 按延迟后的参考结束计算。
默认关闭时保持旧语义。

## 第一组：启动单因素

16 案：detour、开放单段斜航 × 四档速度 × 启动关闭/开启，每案两次。
初始 pitch=0、静止，固定正式恢复系数 6 的控制候选、终点捕获、无流 Physics。
开启启动后均约 8.10 s 释放，这是实际闭环运动，不是预对齐 initial_state。

| 速度 m/s | 原 detour | 开启启动 | 原净裕量 m | 启动后净裕量 m |
| --- | --- | --- | ---: | ---: |
| 0.2 | SUCCESS | SUCCESS | 0.0270 | 0.1054 |
| 0.3 | SUCCESS | SUCCESS | 0.0080 | 0.1402 |
| 0.4 | COLLISION | OUT_OF_BOUNDS | -0.0005 | 0.1771 |
| 0.5 | COLLISION | OUT_OF_BOUNDS | -0.0026 | 0.2109 |

开放斜航全部成功。启动消除了本组首次贴墙碰撞，但不能独立解决高速末段越界。

## 第二组：固定启动，仅改变角点时间参数化

16 案：原 detour（固定启动开启）、开放孤立转弯（固定原进入条件）
× 四档速度 × 恒速/局部减速，每案两次。控制器和终点策略保持不变。

实验 `validation.controlled_startup_experiments.corner_timing`：

- 原 sampled geometry、pitch 阶跃不变，shortcut 关闭。
- 内部节点前后各约 1 m 弧长内速度取 min(nominal,0.2) m/s。
  用采样区间中点决定整段速度，因此区域边界有采样量化误差。
- 同时更新 outgoing `Twist.u` 与区间时间 `length/speed`，终点 Twist 为零。
- 0.2 m/s 组完全等价，作为负对照。

这不是通用转角阈值算法：collinear 内部节点也会触发当前实验规则。
没有加速度限制、提前制动模型或连续 pitch-rate 设计；1 m、0.2 m/s 是试验候选，
并非已识别的最优参数。没有只降 u_ref 而保持原位置推进。

| 速度 m/s | 启动+恒速 | 启动+减速 | 减速总耗时 s | 减速净裕量 m |
| --- | --- | --- | ---: | ---: |
| 0.2 | SUCCESS | SUCCESS | 74.40 | 0.1054 |
| 0.3 | SUCCESS | SUCCESS | 62.20 | 0.1412 |
| 0.4 | OUT_OF_BOUNDS | SUCCESS | 59.55 | 0.1809 |
| 0.5 | OUT_OF_BOUNDS | SUCCESS | 55.10 | 0.2179 |

净裕量是障碍中心距离减 0.30 m，不是边界距离。成功判定与输入限制未放宽。
失败组提前停止，不能用其较短耗时直接比较时间性能。

开放孤立转弯的运动阶段最大路径偏离：

| 速度 m/s | 恒速 m | 局部减速 m |
| --- | ---: | ---: |
| 0.2 | 0.518 | 0.518 |
| 0.3 | 0.719 | 0.527 |
| 0.4 | 0.908 | 0.553 |
| 0.5 | 1.084 | 0.584 |

减速未消除偏离。Detour 0.5 m/s 的同期偏离从 1.135 降至 0.642 m；
最小障碍间隙改善较小，符合“最窄间隙位于转弯前”的诊断。
成功组仍依靠约 19–23 s 的末端捕获，不是精确沿参考到点停车。

## 验证边界

诊断检查覆盖原始初态、参考时钟对应、启动外环旁路、一致限幅、历史长度及重复性。
新增测试覆盖制动先行、限速、连续稳定、预算/漂移/航向拒绝、实际弦段穿障、
释放时积分连续、水平和上下斜线两档速度，以及减速几何/时间/速度一致性。

保留为可选回归候选，不切换默认。未验证强初始速度、不同转角、逆向任务、海流、
参数失配、随机扰动、dt 收敛；未重跑完整原 16 场景基准，不能宣布复杂场景鲁棒安全。
建议下一步固定候选，完成完整场景回归和模型/初态扰动，再决定是否推广。

## 复现与证据

```powershell
.venv\Scripts\python.exe -m validation.controlled_startup_experiments --stage startup --output artifacts/navigation/startup_new_run
.venv\Scripts\python.exe -m validation.controlled_startup_experiments --stage corner --output artifacts/navigation/corner_new_run
```

证据：`artifacts/navigation/controlled_startup_verified_20260917/`、
`artifacts/navigation/corner_slowdown_verified_20260917/`。
保存历史、配置、重复一致性、图、pytest 日志、包版本及 SHA-256。
随后扩展水平/上下斜线参数化测试，最终测试证据为
`artifacts/navigation/startup_corner_final_tests_20260917.xml`；旧 runner 日志不冒充新增测试证据。

最终全仓 **652 passed（18.56 s）**，其中启动/时间参数化专项 13 项。
两组 artifact manifest 均零不匹配；启动证据之后只扩展了该专项测试文件，生产代码
哈希未改变；角点证据记录的源码哈希与当前一致。输出图已目视检查。
