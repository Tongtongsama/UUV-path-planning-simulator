# feasible_v2 的 S1/S2/S3 独立矩阵适配

日期：2026-11-06。入口：`ros2/twin_axial_feasible_matrix`。

本修订复用原 feasible_v2 的 Guide 和 CONFIG 对象，保留原模型、PID、分配器、恒流修正和数值验收阈值。新增 S1 单挡板几何路径适配；S2/S3 使用原双挡板路径生成器。场景几何直接从生成的 SDF 读取并核对。所有水流、初态使用同一套配置，不按工况调参。

## 矩阵与运行顺序

3 场景 × 3 水流 × P0–P4 = 45 个名义条目，每组另加 P0_repeat，共 54 个条目。预检拒绝占据原矩阵条目，不能删去或计为动态通过。Python 记录只证明 Python 参考闭环，不替代 xdyn、通信及 Gazebo 位姿回写验收。

真实运行设有完整 Python 矩阵门禁，并核对当前代码与 Python 源码快照。默认 GUI 开启；只有显式 `--headless` 才关闭。门禁拒绝时不创建真实运行目录，也不启动仿真。

## S3 的限制

S3 障碍为 `[6,7]×[-20,-5.5]` 与 `[13,14]×[-14.5,0]`，水平间隙 6 m、垂向重叠 9 m。规划净距保持 1.5 m。

对前向单调且斜率受限的路径，必要下降路径角下界为：

`atan2(9,6) + asin(2×1.5 / sqrt(6²+9²)) = 72.412°`。

当前稳态路径证书在 F0 下最多支持 70°；F2 向上流在巡航速度 0.4 m/s 下最多约 67.552°，即使增加到既有速度上限 0.6 m/s 也仍小于 70°。单纯降速不能解决这一向上流下降段限制。F1 的地速方向与艇体方向不同，不能直接套用 F0 的 70° 路径角上限。

这是当前路径族/稳态可执行性证书的限制，不是“任何非线性瞬态机动都不可能”的证明。不得通过缩短 S3 挡板、降低净距、放宽俯仰或跟踪误差阈值来宣称原矩阵完成。

## 启动与复核

Windows PowerShell：

```powershell
cd C:\Users\Songyuan\uuv_simulator
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible_matrix.cli preflight --output artifacts/twin_axial_feasible_matrix/preflight_new
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible_matrix.cli python-matrix --output artifacts/twin_axial_feasible_matrix/python_new
```

输出目录必须不存在。存在拒绝或失败时退出码为 2，记录仍保留。首次 `python_20261106_01` 因新增汇总器的 NumPy 布尔 JSON 序列化错误中断，保留作开发记录；正式修正版使用 `python_20261106_02`，不能混用源码快照。

只读复核正式记录（会重算动力学和控制响应，不启动 Gazebo）：

```powershell
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible_matrix.cli python-analyze artifacts/twin_axial_feasible_matrix/python_20261106_02
```

进入 NixOS：

```powershell
wsl -d NixOS
```

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash ros2/twin_axial_feasible_matrix/run-nix.sh real-matrix \
  --python-matrix artifacts/twin_axial_feasible_matrix/python_20261106_02 \
  --output artifacts/twin_axial_feasible_matrix/xdyn_new --check-only
```

当前矩阵未通过时，预期输出 `passed: false`、`real_started: false`。以后只有完整 Python 矩阵通过，才可去掉 `--check-only` 执行默认 GUI 的真实矩阵。真实记录复核入口为 `run-nix.sh analyze <真实矩阵目录>`。

## 保护与测试

`artifacts/twin_axial_feasible_matrix/preservation_20261106.json` 核对 66 个受保护源码与 22,507 个既有实验文件，全部哈希未变。`regression_20261106.xml`：1030 tests passed。新场景测试覆盖原引导对象复用、S2 路径一致、SDF 几何、S3 限制、拒绝记录防篡改、真实门禁和验收阈值未变。

## 本轮结果

正式记录：`artifacts/twin_axial_feasible_matrix/python_20261106_02`。简表：`artifacts/twin_axial_feasible_matrix/results_20261106.json`。以下误差/净距为包含重复试验在内的组内最坏值，单位 m。

| 场景/水流 | Python 结果 | 最大误差 | 最大 RMS | 最小安全净距 |
|---|---|---:|---:|---:|
| S1/F0 | 6/6 通过 | 0.211 | 0.063 | 1.900 |
| S1/F1 | 6/6 通过 | 0.122 | 0.063 | 1.854 |
| S1/F2 | 6/6 通过 | 0.444 | 0.174 | 1.872 |
| S2/F0 | 6/6 通过 | 0.232 | 0.113 | 1.507 |
| S2/F1 | 6/6 通过 | 0.221 | 0.100 | 1.505 |
| S2/F2 | 6/6 通过 | 0.223 | 0.114 | 1.482 |
| S3/F0 | 6 个预检拒绝 | — | — | — |
| S3/F1 | 6/6 通过 | 0.248 | 0.116 | 1.457 |
| S3/F2 | 6 个预检拒绝 | — | — | — |

7 个已运行组的 P0 重复性均通过。共 35 个名义动态条目及 7 个重复动态条目通过；10 个名义条目及 2 个重复条目预检拒绝。拒绝组的 `all_safe: false` / `repeatability: false` 表示没有动态证据，不代表已发生碰撞或重复性失效。

**完整矩阵没有通过，不能宣布 S1/S3 的真实闭环验收完成。** 本轮未启动真实运行；此前 S2 的 feasible_v2 真实证据仍属于原版本，不冒充本独立修订的真实矩阵结果。

正式 `matrix.json` 的 `cross_scene_source_identical: true`：54 个条目使用相同源码。NixOS 实际执行 `real-matrix --check-only` 后返回 `passed: false, real_started: false`，原因是完整 Python 矩阵未通过；未创建 `xdyn_gate_20261106` 目录。日志在 `artifacts/twin_axial_feasible_matrix/real_gate_20261106.log`。此拒绝是预期的门禁行为，不是运行环境故障。

## 下一步决策

在保留原 S3 和全部阈值的前提下，需要先单独研究超出当前稳态路径证书的动力学可行轨迹/引导方法，并证明其跟踪余量，再决定是否修改新的独立引导修订。当前几何下，仅调整前向单调路径形状或降低速度无法消除上述证书障碍。

另一种论文范围选择是明确将 S3/F0、F2 定义为适用边界案例，而非名义成功场景；这会改变论文矩阵的解释，必须另行明确决定。本次没有改写原场景、完成标准或验收矩阵以使其通过。

后续 S3 瞬态小规模研究见 [S3 瞬态可行性](lotusim_twin_axial_s3_transient.md)。12 组统一引导参数中没有任何一组同时通过 S3/F0、F1、F2；F0/F1 可用的低前视组在 F2 触发安全线，F2 唯一通过的高前视组又使 F0/F1 越界。因此本轮没有进入完整矩阵，S3 仍保持名义场景定位。
