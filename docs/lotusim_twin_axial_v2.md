# uuv_twin_axial_v2：附加质量耦合修订

这是独立目录 `ros2/twin_axial_v2/` 中的模型修订。原 `ros2/twin_axial/` 和 `uuv_twin_axial_v1` 原样保留；原 Python 动力学、控制器、规划算法与冻结数据不变。未接 PID。

## 模型修订

锁定版本的 xdyn 已自动计算刚体 Coriolis 力，但当前 v1 外力配置遗漏附加质量 Coriolis 力。v2 通过 xdyn 的 maneuvering 表达式外力接口，在积分阶段读取当前机体系速度，加入 `-C_A(nu)nu`。没有修改 xdyn 源码，也没有通过 ROS 发布延迟补偿力。

无流、FRD 机体系，`nu=(u,v,w,p,q,r)`，附加质量 `M_A=diag(5,5,8,2,2,2)`。新增六维外力为：

```text
X =  5*v*r - 8*w*q
Y =  8*w*p - 5*u*r
Z =  5*u*q - 5*v*p
K = -3*v*w
M =  3*u*w
N =  0
```

推导使用 `a=diag(5,5,8)*(u,v,w)`、`b=2*(p,q,r)`，外力是 `[-omega×a; -velocity×a-omega×b]`。因此 `nu·force=0`，该耦合项不产生机械能。纵向子空间退化为 `(-8*w*q, 5*u*q, 3*u*w)`，与原 Python 的 `-C_A(nu)nu` 一致。

新增 Z 是被动水动力耦合，不是新执行器。可控输入仍只有上、下两个轴向推进器：`X=Tu+Tl, M=.25*(Tl-Tu)`，各 ±15 N，分配可行域 `|X|+4|M|≤30`。阻尼、恢复力、质量、时间步长和分配逻辑保持 v1 数值。

本修订只支持本轮无流验证。加入洋流时须统一相对速度、附加质量及流速导数处理，不能直接用绝对机体速度表达式宣称含流模型已验证。六维公式经过代数测试；真实求解器响应对照范围是纵向运动，不代表任意六自由度工况均已验证。

## Step by step

1. 打开 PowerShell，输入：

```powershell
wsl -d NixOS
```

2. 进入项目：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
```

3. 运行数值检查。不需要提前启动 LOTUSim 或 Gazebo：

```bash
bash ros2/twin_axial_v2/run-nix.sh numerics \
  --output "artifacts/twin_axial_v2_numerics/check_$(date +%Y%m%d_%H%M%S)"
```

依次检查 0.05、0.025、0.0125 s 步长，最后打印 JSON。使用真实 `xdyn-for-cs`，不打开 GUI。端口 12358 必须空闲。

4. 完成后运行 LOTUSim 开环验收：

```bash
bash ros2/twin_axial_v2/run-nix.sh suite \
  --output "artifacts/twin_axial_v2/suite_$(date +%Y%m%d_%H%M%S)"
```

脚本自动启动、停止其自己的 xdyn、网关和 LOTUSim，默认无 GUI。依次运行静止、正负前进、正负俯仰、分配饱和、断流、过期时间戳及重复前进，共九次。让程序自然结束，不要中途 Ctrl+C。

开环入口沿用端口 12356/12357、ROS domain 74；不要与 v1 或另一份本试验并行启动。脚本遇到占用会报错，不会关闭其他进程。每次输出路径必须全新。

5. 如只想运行一次俯仰试验：

```bash
bash ros2/twin_axial_v2/run-nix.sh run --case pitch_positive \
  --output "artifacts/twin_axial_v2/pitch_$(date +%Y%m%d_%H%M%S)"
```

## 怎么验收

数值报告 `acceptance.json` 顶层 `passed: true`，并检查 `energy`、`convergence`、`consistency`、`coupling_diagnostic.passed` 和 `clean_shutdown`。原 M4/M5/M6 阈值不变；增加更严格的修订检查：各步长 Python/xdyn 最大位置差 ≤1e-6 m、俯仰差 ≤1e-5°、u/w 速度差及 q 差 ≤1e-6（分别为 m/s、rad/s）。这些是此固定短试验的对照阈值，不是实艇准确度。

数值输入沿用：0–2 s 零、2–7 s 上下各 2 N、7–12 s 上 -1/下 +1 N、12–22 s 零。能量比较撤除输入前后 10 s，要求末值 ≤初值的 101%；收敛在 20 s 对比，位置/俯仰/速度限值为 .02 m/.5°/.01 m/s；原响应容差为 .05 m/1°/.02 m/s。

开环检查最终 `suite.json` 的 `passed: true`，九个运行退出码为 0，重复性通过；各子目录 `acceptance.json` 顶层通过，包含时间连续、姿态映射、方向、执行器限幅、归零及记录哈希检查。断流/过期试验中“归零”指推进器输入归零，不要求艇体瞬间停止，也不要求被动阻尼和耦合力归零。

只读复核本次已完成的记录，不重跑动力学、不修改记录：

```bash
bash ros2/twin_axial_v2/run-nix.sh numerics-analyze \
  artifacts/twin_axial_v2_numerics/validation_20261026_01

bash ros2/twin_axial_v2/run-nix.sh analyze \
  artifacts/twin_axial_v2/validation_20261026_01/pitch_positive
```

第二条可将 `pitch_positive` 换成其他用例目录。记录包括源文件快照、依赖锁定、模型配置、完整状态、原始通信、验收报告及哈希清单。旧记录仍用旧入口复核。

## 下一步

数值和开环通过后再适配现有 PID 与分配后抗饱和，依次验收前进速度、正负俯仰、无障碍直线和深度变化，最后接 LOS 和规划路径。本轮完成的是模型修订与开环验证，不是 PID 闭环。

## 2026-10-26 实测结果

以上两条 `validation_20261026_01` 路径是已完成的真实运行记录，均通过。只读复核确认全部记录哈希及源文件快照匹配。

| 项目 | 实测 |
|---|---|
| 三步长能量检查 | 全通过；dt=.05，撤除输入后 10 s 从 0.6661642096 降至 0.02550008258 J |
| xdyn .05→.025 末端差 | 位置 2.73e-10 m，俯仰 1.89e-7°，速度 1.91e-10 m/s |
| xdyn .025→.0125 末端差 | 位置 2.07e-11 m，俯仰 1.20e-8°，速度 1.18e-11 m/s |
| 原 Python／修订 xdyn，dt=.05 最大差 | 位置 9.32e-11 m，俯仰 3.43e-10°，速度 7.63e-13 m/s，q 5.85e-12 rad/s |
| 九次 LOTUSim 开环 | 全通过；静止 600 步，其余每次 160 步，Gazebo 姿态观测数分别相同 |
| 重复前进 | 对齐 160 步，位置和机体速度差均为 0 |
| 相关单元/回归测试 | 159 passed |

严格耦合响应门限也在三个步长全部通过。仅在诊断包装器中去掉 Python 附加质量耦合后，位置差重新达到约 .02246 m、俯仰差约 .36118°，支持本修订确实消除了此前定位的缺项。正式验收始终使用未修改的 Python 参考。

49 个原模型/算法/配置文件以及本轮额外保护的 11 个 v1 文件哈希未变；冻结回放数据校验通过，原 v1 数值记录哈希仍有效。没有覆盖旧输出或改变原验收容差。
