# feasible_v1：S2 三流同配置对照（2026-11-02）

本轮只运行对照与新增启动编排，不修改feasible_v1引导/路径配置、模型、PID、分配器、冻结数据或验收阈值。三种水流的Python源码快照65个文件完全一致，P0路径和配置完全一致；F2使用2026-11-01已验收记录。

## 最终汇总

本轮对照已结束，结论不是“三流全部通过”。F0在Python门槛失败，真实试验未启动；F1真实GUI六次全部通过；F2沿用同配置已通过的真实记录。

| 水流 | Python五初态 | Python最大RMS / 最大误差 m | LOTUSim/xdyn | 真实最小净距 m | 真实最大RMS / 最大误差 m |
|---|---|---|---|---:|---|
| F0 (0,0) | 0/5通过，全部安全到达 | 0.37646 / 1.00210 | 未运行：Python门槛失败 | — | — |
| F1 (-0.05,0) | 5/5通过 | 0.21873 / 0.70501 | 5/5通过，P0重复通过，GUI开启 | 1.46609 | 0.21826 / 0.70391 |
| F2 (0,+0.05) | 5/5通过 | 0.25072 / 0.66123 | 5/5通过，P0重复通过，沿用无GUI记录 | 1.50638 | 0.24974 / 0.65971 |

F1真实目录：`artifacts/twin_axial_feasible/xdyn_f1_gui_20261102_01`。

| 初态 | 到达时间 s | 最小净距 m | RMS m | 最大路径误差 m |
|---|---:|---:|---:|---:|
| P0 | 135.90 | 1.47882 | 0.20655 | 0.67807 |
| P1 | 135.70 | 1.47911 | 0.20312 | 0.66947 |
| P2 | 136.20 | 1.47513 | 0.20956 | 0.68639 |
| P3 | 134.65 | 1.46609 | 0.21826 | 0.70391 |
| P4 | 135.75 | 1.49058 | 0.19720 | 0.65376 |
| P0_repeat | 135.90 | 1.47882 | 0.20657 | 0.67807 |

F1全部六次：原27项验收检查通过，分配饱和0%，Gazebo位姿覆盖100%，正常清理与最终归零通过。P0重复2721个匹配状态位置差0 m、到达时间差0 s；结束后记录长度相差0.05 s，低于原0.10 s标准，不能说两个原始日志完全逐字节相同。

F1与F2真实源码快照完全一致。F1本轮开启GUI，F2沿用之前无GUI的正式记录，这个显示运行方式差异已明确记录，未修改仿真时间步、控制参数或阈值。Python三流使用同一配置和同一后端，适合定位F0的起步跟踪问题。

最终证据：`artifacts/twin_axial_feasible/comparison_20261102/summary.json`（三流指标及suite文件SHA-256）、`python_comparison_integrity.json`、`python_diagnosis.json`、`protected_before.json`、`f0_real_gate.json`和`gate_tests.xml`。72个原有模型/控制/导航等文件哈希未变；共2640项三流Python及F1/F2真实运行记录文件校验无错误。未改动旧封存或旧验收结果。

## Python结果与F0门槛

- F0：`artifacts/twin_axial_feasible/python_f0_20261102_01`。五初态0/5通过，P0重复得到相同失败。全部安全到达；五例最大路径误差均超过0.8 m，P0–P3的RMS也超过0.35 m。
- F1：`artifacts/twin_axial_feasible/python_f1_20261102_01`。五初态5/5通过，P0重复通过，全部安全。
- F2：`artifacts/twin_axial_feasible/python_20261101_01`。五初态5/5通过，P0重复通过，全部安全。

F0/P0在11.95 s首次超过0.8 m；17.40 s达到最大误差0.953937 m，位置约(4.2903,-7.2666)，实际俯仰70.398°、目标70°。这是起步上升段的跟踪问题，不是第二挡板附近的安全触线。仅凭此不能断言每项改动的因果贡献，但足以否定“feasible_v1三流均已通过”。不将失败替换为旧版本F0的通过结果。

按先通过Python再进入真实验收的顺序，F0未启动LOTUSim/xdyn。`artifacts/twin_axial_feasible/comparison_20261102/f0_real_gate.json`记录实际门槛检查：退出码2，`real_started=false`，未创建真实输出目录。它是未进入真实验证，不是xdyn实测失败。

## 新增编排入口

旧 `ros2/twin_axial_feasible/cli.py` 的门槛固定检查F2。为保持原包源码及已验证快照不变，本轮新增：

- `scripts/run_feasible_flow_suite.py`
- `scripts/run_feasible_flow_suite.sh`

新入口使用原Python分析器重算所选水流的套件，验证水流匹配、全套通过、源码与当前一致，再调用原真实trial及acceptance。GUI默认开启，可显式`--headless`关闭。它不改任何导航或动力学算法，不改变4/5、P0必过、全部安全和重复性要求。新增门槛测试4项通过。

## 如何运行

PowerShell中先运行Python参考套件：

```powershell
cd C:\Users\Songyuan\uuv_simulator
$trialTag = Get-Date -Format yyyyMMdd_HHmmss
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible.cli python-suite --flow F1 --output "artifacts/twin_axial_feasible/python_f1_$trialTag"
```

F0对照把`--flow`改为`F0`并使用新目录即可。当前配置预期未通过；保留结果，不调阈值或重试替换。

进入NixOS：

```powershell
wsl -d NixOS
```

运行F1真实GUI套件（默认P0–P4加P0_repeat，每次结束窗口会关闭并为下一初态重新打开）：

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash scripts/run_feasible_flow_suite.sh \
  --flow F1 \
  --python-suite artifacts/twin_axial_feasible/python_f1_20261102_01 \
  --output "artifacts/twin_axial_feasible/xdyn_f1_gui_$(date +%Y%m%d_%H%M%S)"
```

如果自行重新生成了Python套件，替换`--python-suite`为实际目录。真实试验期间不要按Gazebo暂停或关闭窗口；需要交互观看建议使用已完成记录的离线回放。不要同时启动另一套占用12356/12357端口及ROS domain 74的动力学试验。

## 如何只读复核

PowerShell：

```powershell
cd C:\Users\Songyuan\uuv_simulator
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible.cli python-analyze artifacts/twin_axial_feasible/python_f0_20261102_01
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible.cli python-analyze artifacts/twin_axial_feasible/python_f1_20261102_01
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible.cli python-analyze artifacts/twin_axial_feasible/python_20261101_01
```

F0应为`passed=false`、`successful_initials=0`、`all_safe=true`，F1/F2应为`passed=true`、`successful_initials=5`。失败命令退出非零是正确结果。

NixOS复核真实套件：

```bash
bash ros2/twin_axial_feasible/run-nix.sh analyze artifacts/twin_axial_feasible/xdyn_f1_gui_20261102_01
bash ros2/twin_axial_feasible/run-nix.sh analyze artifacts/twin_axial_feasible/xdyn_suite_20261101_01
```

这些分析命令不打开GUI、不重跑动力学、不修改记录。无需对只有门槛拒绝而不存在的F0真实目录运行分析。

## 后续范围

本轮不调参。下一修订若要满足三流统一验收，应先单独分析初始上升段中姿态建立与加速的协调，保留本轮F0失败基线；任何新修订都必须重新验证三流，不能仅保留本轮F1/F2旧通过作为新版本的证明。
