# uuv_twin_axial_v1 无流数值验收

本入口检查机械能、步长收敛和 Python/xdyn 响应容差，不接 PID，不修改原模型、算法或冻结记录。它直接调用锁定的真实 `xdyn-for-cs`，通过 WebSocket 交换输入与完整状态；不启动 Gazebo 或 ROS。因此这是求解器层验证，不能代替此前 LOTUSim 通信验收，也不能证明闭环控制已完成。

## 启动

1. Windows 打开 PowerShell：

```powershell
wsl -d NixOS
```

2. 进入项目并运行（每次使用新的输出目录）：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
bash ros2/twin_axial/run-nix.sh numerics \
  --output "artifacts/twin_axial_numerics/check_$(date +%Y%m%d_%H%M%S)"
```

不需要另开 LOTUSim/UI。端口 12358 必须空闲；不要同时运行两份本检查。程序依次打印 `dt=0.05`、`0.025`、`0.0125`，最后打印 JSON。成功退出码为 0；检查失败为非零。不会出现 Gazebo 窗口。

3. 只读取已完成记录并重新计算验收，不启动动力学：

```bash
bash ros2/twin_axial/run-nix.sh numerics-analyze \
  artifacts/twin_axial_numerics/validation_20261026_final
```

检查自己新生成的结果时，把最后一行换成该次实际输出目录。已有目录不会被覆盖；失败记录也保留。

## 固定试验

无流、初始静止，ENU 位置 (2,0,-30)，朝东，俯仰为零。两推进器输入为牛顿，正负号沿用独立开环验收。Python 使用原有非线性 3-DOF 模型和 RK4；xdyn 使用当前 twin axial 配置及锁定依赖。

| 仿真时间 | 上推进器 | 下推进器 | 作用 |
|---|---:|---:|---|
| 0–2 s | 0 | 0 | 静止 |
| 2–7 s | 2 | 2 | X=4 N |
| 7–12 s | -1 | 1 | M=0.5 N·m |
| 12–22 s | 0 | 0 | 自由衰减 |

三种步长均与输入切换时刻对齐。响应对比使用 0–20 s；能量检查延伸到 22 s，以取得撤除输入后的完整 10 s。

## 验收

| 检查 | 预先固定的标准 |
|---|---|
| M4 无流机械能 | E(22) ≤ 1.01 E(12)，分别检查两后端及三个步长 |
| M5 步长收敛 | 20 s 末端相邻步长差：位置 ≤0.02 m、俯仰 ≤0.5°、速度 ≤0.01 m/s；第二次减半误差不增加，允许报告中的数值噪声底限 |
| M6 无流响应容差 | 对齐时刻最大差：位置 ≤0.05 m、俯仰 ≤1°、u/w 速度 ≤0.02 m/s；检查 xdyn 平面外状态 |

机械能包括总惯性动能及恢复力势能。纵向形式为 `0.5*(25*u²+28*w²+10*q²)+6*(1-cos(theta))`；xdyn 额外计入平面外动能和横滚恢复势能。另记录撤除输入后的最大能量及角速度响应误差。

最终 `passed: true` 表示上述有限容差检查及进程退出检查通过。只读复核还须有 `recording_hashes: true`。**它不表示两套方程完全等价，也不代表含流 M6、PID 或导航验收已通过。**

## 保存内容与已知差异

- `acceptance.json`：各项指标、阈值和耦合项诊断。
- `protocol.json`、`source/`、`flake.lock`：输入协议、源文件快照和依赖锁定。
- `states_*.json`：原 Python 与真实 xdyn 的状态序列。
- `dt_*/transactions.jsonl`、`model.yml`、`xdyn.log`、`process.json`：原始通信、配置、求解器日志和退出记录。
- `manifest.json`：记录哈希；复核不改原始记录。
- `diagnostic_reference_no_added_coriolis.json`：仅用于定位模型差异，绝不替换正式 Python 参考或用于通过 M6。

当前 xdyn 配置缺少原 Python 模型中的附加质量 Coriolis 耦合力；减小步长不能消除这一模型差异。诊断只在测试包装器中抵消该项，原算法与模型没有修改。详细结果见 [本次实测报告](lotusim_twin_axial_numerical_results_20261026.md)。接 PID 前，应先在独立版本中补齐 xdyn 耦合项并重跑本检查与开环验收。
