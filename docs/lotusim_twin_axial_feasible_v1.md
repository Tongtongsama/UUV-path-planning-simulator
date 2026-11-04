# S2/F2 独立可执行路径与流补偿修订

日期：2026-11-01。入口 `ros2/twin_axial_feasible/`，配置ID `feasible_v1`。

本修订不改原导航包、v2模型、恒流受力、PID增益、双推进器分配、保护协议、旧封存内容或验收阈值。新增入口是独立修订，旧入口仍运行原方案。不得将本修订记录混入旧源码冻结套件。

## 2026-11-01 完成结果

Python：`artifacts/twin_axial_feasible/python_20261101_01`；真实LOTUSim/xdyn：`artifacts/twin_axial_feasible/xdyn_suite_20261101_01`。两套均五初态5/5通过，加P0重复通过；真实六次各27项检查全部通过。没有失败重试替换样本。

| 初态 | Python到达 s | Python RMS m | xdyn到达 s | xdyn最小净距 m | xdyn RMS m | xdyn最大误差 m |
|---|---:|---:|---:|---:|---:|---:|
| P0 | 133.25 | 0.23289 | 133.30 | 1.50971 | 0.23224 | 0.62410 |
| P1 | 132.95 | 0.22670 | 133.00 | 1.50872 | 0.22619 | 0.61133 |
| P2 | 133.50 | 0.23877 | 133.60 | 1.51034 | 0.23829 | 0.63665 |
| P3 | 133.30 | 0.25072 | 133.25 | 1.51152 | 0.24974 | 0.65971 |
| P4 | 133.20 | 0.21594 | 133.25 | 1.50638 | 0.21549 | 0.59128 |
| P0_repeat | 133.25 | 0.23289 | 133.30 | 1.50971 | 0.23224 | 0.62410 |

真实全部运行：分配饱和比例0%，Gazebo位姿回写覆盖100%，正常清理与最终输入归零通过，源码快照一致。P0重复2670个匹配状态位置差0 m、到达时间差0 s。Python重复同样一致。两种后端到达时间有0.05–0.10 s量级差异；本表是各自闭环验收，不冒称新增固定输入M6配对证明。

单次P0诊断：实际最大俯仰72.086°、最大|q|=0.2373 rad/s、单台最大推力14.141 N，均在原包络内。原始俯仰请求约26.9%触及目标限幅，不能宣称全程无限幅；这与执行器分配饱和比例是不同指标。

全量回归1011项通过（`artifacts/twin_axial_feasible/regression_20261101.xml`）。272个原源码/配置/文档文件哈希未变，1962项旧实验文件哈希复核通过；独立历史里程碑原脚本519项校验通过。新结果汇总位于 `artifacts/twin_axial_feasible/validation_summary_20261101.json`，旧封存没有加入这些新运行。

现有结果的只读复核命令：

```powershell
cd C:\Users\Songyuan\uuv_simulator
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible.cli python-analyze artifacts/twin_axial_feasible/python_20261101_01
```

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash ros2/twin_axial_feasible/run-nix.sh analyze artifacts/twin_axial_feasible/xdyn_suite_20261101_01
```

静态图 `artifacts/twin_axial_feasible/s2_f2_p0_comparison_20261101.png` 展示旧失败与新通过P0轨迹、跟踪误差及速度安排。这是路径/引导联合修订对照，不是分项消融试验。

## 实现范围

- 从实际 SDF 解析两挡板几何。新增生成器只支持当前交替贴底/贴顶的双矩形挡板拓扑，不是通用规划器，也不冒称修改后的 A*。旧 A* 保留；这里使用由障碍角点、1.55 m切线净距及外侧转向空间生成的候选路径。
- 使用二次Bezier转角过渡及保留顶点的密集折线；逐整段检查障碍和边界净距≥1.5 m，检查起终点。
- 在0.4 m/s巡航条件下，逐段求恒流配平，拒绝超过70°目标俯仰或±15 N执行器范围的候选。该检查不等同瞬态可行性保证；完整动力学验证仍是必要门槛。
- 使用1 m前视点提前转向，2 m切向变化预览决定0.12–0.4 m/s分段目标速度，目标速度变化率≤0.04 m/s²。
- 根据状态反馈的艇体横向/升沉速度补偿对地航迹角与艇体俯仰差异。记录已知水流的艇体系分量和剩余瞬态分量；两者之和即实际w。它是状态反馈漂移补偿，不是新增水流观测器，也不向位置运动学重复添加水流。
- 目标俯仰仍限制±70°，变化率≤0.12 rad/s，并向原PID的既有q参考接口提供该变化率。PID本体/参数/抗饱和/分配完全复用。目标角限幅事件完整记录；不承诺所有瞬态指令都无需限幅。
- 真实入口复用旧ROS/网关/xdyn启动和清理流程的独立副本，新增修订源码快照。只使用实际状态反馈与推力输入，不使用MOVE或轨迹回写代替动力学。

## 验收与门槛

Python入口记录完整状态、实际广义力、控制诊断、路径、配置、源码和manifest。只读分析器重新积分每一步，并重放引导/PID/分配，重新计算完整任务指标。验收保持300 s内、0.5 m内持续2 s；扫掠净距≥0.9 m；全程RMS≤0.35 m、最大误差≤0.8 m；饱和比例≤20%；实际俯仰≤75°、速度≤1.5 m/s、q≤0.8 rad/s。

Python套件包含P0–P4和P0_repeat，要求P0通过、五初态至少4/5、全部安全、重复一致、源码一致。真实入口必须传入该套件路径：启动前只读重算门槛，逐文件核对当前代码与Python源码快照一致。未通过或代码有变时拒绝启动真实试验。

Python仅证明参考模型纵向闭环，不包含真实六自由度平面偏移、ROS时序/断流及Gazebo位姿回写。真实验收保留原导航分析器的数值阈值、模型一致性、逐消息控制重放、执行器输入验证、平面包络、≥95%位姿回写覆盖和最终归零要求。仅适配新Mission与源码快照路径，没有放宽阈值。

## 启动方法

### 1. PowerShell：Python参考模型

```powershell
cd C:\Users\Songyuan\uuv_simulator
$trialTag = Get-Date -Format yyyyMMdd_HHmmss
$pythonRun = "artifacts/twin_axial_feasible/python_$trialTag"
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible.cli python-suite --output $pythonRun
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible.cli python-analyze $pythonRun
```

末尾应有 `passed: true`、`successful_initials: 5`、`all_safe: true`、`repeatability: true`、`source_consistent: true`。每次必须使用新目录；失败不会自动重试替换。

### 2. PowerShell进入NixOS

```powershell
wsl -d NixOS
```

### 3. NixOS：真实xdyn/LOTUSim单次

以下路径指向本次已验证的Python套件。若自己重新运行，替换为上一步实际生成的相对路径。

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash ros2/twin_axial_feasible/run-nix.sh run \
  --flow F2 --case P0 \
  --python-suite artifacts/twin_axial_feasible/python_20261101_01 \
  --output "artifacts/twin_axial_feasible/xdyn_p0_$(date +%Y%m%d_%H%M%S)"
```

默认无GUI，避免显示卡顿。需要观察时可显式增加 `--gui`，但画面不是验收依据。不要同时运行其他占用12356/12357端口或ROS domain 74的试验；本入口不会结束其他程序。

### 4. 真实五初态与重复试验

```bash
bash ros2/twin_axial_feasible/run-nix.sh suite \
  --flow F2 \
  --python-suite artifacts/twin_axial_feasible/python_20261101_01 \
  --output "artifacts/twin_axial_feasible/xdyn_suite_$(date +%Y%m%d_%H%M%S)"
```

`suite.json` 应显示 `passed`、`all_runs_safe`、`source_snapshot_consistent`、`repeatability.passed` 为true。查看各 `acceptance.json` 的完整checks，不只看是否到达。

### 5. 只读复核真实记录

```bash
bash ros2/twin_axial_feasible/run-nix.sh analyze artifacts/twin_axial_feasible/<实际运行目录>
```

分析不会重启动力学或改写记录。退出码0为通过，2为未通过；错误日志和失败记录保留。Nix脚本使用已有锁文件及离线缓存；新电脑缺缓存时先按项目现有LOTUSim/Nix环境安装步骤准备依赖，不要改锁文件绕过版本约束。

## 范围边界

当前目标仅为S2/F2；F0/F1接口是后续同配置对照入口，未经实际运行不能声称已通过。未扩展到S1/S3、完整45次矩阵、未知/时变水流或真实载具。新生成器对拓扑有明确限制，应在论文中如实说明，不把这一实现称为通用最优路径规划算法。
