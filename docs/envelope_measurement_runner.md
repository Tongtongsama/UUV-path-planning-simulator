# Step 2 — Deterministic stopping / turning runner

日期：2026-09-20。仅 measurement harness；无 Planner、terminal production policy、
corner slowdown 或 PID 改动。依据 stopping_turning_envelope_experiment_design.md。

## 冻结配置和 schema

`config/envelope_measurement_v1.json` 与 `config/envelope_measurement.schema.json`
构成 exact-value 预注册配置。Schema 采用 JSON Schema 的 closed object / const 子集，
runner 内置校验该子集，不引入 jsonschema 运行依赖。新增键、缺项、布尔冒充数字、
非有限数或参数变化均拒绝。改实验需新增版本，而非编辑结果后回填 v1。

- dt 0.05 s；每配置独立重建控制器/物理状态，重复两次。
- 3 速度 × 5 转角幅值 × 2 方向 × 5 海流 = 150 转弯配置。
- 稳定带：cross-track 0.1 m、wrapped pitch 0.08 rad、|q| 0.03 rad/s、
  body surge 与名义速度之差 0.05 m/s，连续 1 s。
- 0.35 rad/s 仅记录超限时间，不终止，不是 Planner 拒绝标准。
- 水平入段 10 m，出段 30 m；角点 (0,-40)，测量到出段投影 8 m；
  开放矩形 [-60,60] × [-100,20]，侵蚀 0.1 m。测量预算 180 s。
- 在 x=-2 m 处冻结入段稳定性标签；此时尚未进入 0.5 m LOS 前视角点区。
  标签失败不调参、不延长，也不抹去其后观测，只排除于稳定入口包络。

## 测量语义

直接调用已有 compute_los → BodyGuidance3DOF → controller → physics。
不构造 Navigator，不调用 startup/capture，也不伪造导航 SUCCESS。
cruising fixture 初始 u 为名义速度、其余速度零，pitch=0、PID 记忆零；
这不是完整任务启动能力实验。入段瞬态计入完整历史和全窗 RMS，不能当成纯角点 RMS。

完成是顺序进度已进入出段且出段投影至少 8 m；终点仍在 30 m，
若意外 terminal_ready 则删失，绝不进入 terminal。
停止/恢复事件用固定采样点确认，不插值：开始样本到确认样本间需满足完整保持时间。
d_entry 从首次持续离开入段稳定带的位置计算，d_exit 从首次持续恢复出段稳定带的位置计算。
未离开为 NOT_OBSERVED；未恢复为 RIGHT_CENSORED，不能用 0 填充。
恢复事件可删失，即使车辆已越过测量结束面；应逐字段读取而非只看 measurement status。

曲率由 world 速度的中心差分（内部点）计算，首尾剔除；低于 0.03 m/s
或曲率不大于 1e-4 /m 的点不参与 radius 分位数，另报有效样本比例。
半径统计是整个测量窗内有效曲率样本的统计，不是单圆拟合或转弯最小能力保证。
零流 ±30°/±90°、0.2/0.5 m/s 共 8 配置另用 dt/2 重复验证敏感性。

虚拟 corridor 使用两条有限线段距离的最小值。每个实际步的线性扫掠按 ≤2 mm
细分，取样本最大值为下界；加半个最大采样间距为 Lipschitz 上界。
上界不超过额外 margin 为 PASS_BOUND，下界超过为 FAIL_WITNESS，中间为 UNRESOLVED。
这是围绕整条有限 polyline 的 tube，非矩形障碍精确再运行；vehicle radius 与
原 safety margin 已留在参考 tube 之外，extra_margin 不再次包含它们。
固定 margin 网格 0/0.025/0.05/0.1/0.2/0.4/0.8/1.6 m。

## Stopping fixture

每种 speed/current 先固定 60 s 水平 inner-loop warm-up，不因未稳定而增加预算。
warm-up 稳定性针对姿态/q/surge（cross-track 不参与漂流 warm-up），不声称位置保持。
随后重置世界位置/时间以统一开放场地，保留实际速度与完整 PID 记忆。
在平移不变的恒流模型中，这不改变动力学；历史单独保存 warmup 与 measurement。

基准入口加单因素扰动：pitch ±15°、q ±0.2 rad/s、w ±0.05 m/s，
不是三者全组合。15 speed/current × 7 入口 = 105 配置；另加保存的原 detour
入口及其精确恢复的 PID 记忆，共 106 配置。扰动标签与 warm-up 稳定标签独立。
保存入口所依赖的 common/los 原始文件纳入源码/输入校验和及快照。

使用现有制动命令 u_target=-clip(u,-0.15,0.15)，保持入口实际 pitch、q_ref=0；
这是诊断制动律，不是新生产 policy。持续 0.5 s 满足对地 speed≤0.03 与 |q|≤0.03
才完成；预算 30 s。未停稳的 stopping_distance 为 null，并标注右删失。
仍保存观察到的最大前移、横移、位移和弧长，不能将这些当作已完成停止距离。

## 运行顺序和证据

```powershell
.\.venv\Scripts\python.exe -m validation.envelope_measurement --stage pilot --output artifacts/envelope_pilot_v1_verified
# 审核 pilot summary/audit 与逐步记录后，才运行以下命令。
.\.venv\Scripts\python.exe -m validation.envelope_measurement --stage full --reviewed-pilot artifacts/envelope_pilot_v1_verified --output artifacts/envelope_turning_v1
.\.venv\Scripts\python.exe -m validation.envelope_measurement --stage stopping --reviewed-pilot artifacts/envelope_pilot_v1_verified --output artifacts/envelope_stopping_v1
.\.venv\Scripts\python.exe -m validation.envelope_measurement --stage refinement --reviewed-pilot artifacts/envelope_pilot_v1_verified --output artifacts/envelope_refinement_v1
```

输出目录必须不存在。每个 repeat 保存完整 JSON gzip history；summary 汇总状态、
删失和指标；audit 检查时间/历史对齐、限幅、进度与无 terminal。
manifest 绑定配置、schema、所有输出、完整源码/测试和依赖输入快照。
跨 stage 必须使用相同 source manifest；重复比较完整有限 JSON 内容，而非仅终点。
两次重复失败会保留证据并停止，不输出合格 manifest。
仿真历史不含 wall-clock；snapshot ZIP 时间戳不参与重复仿真等价比较。

初期 `artifacts/envelope_pilot_v1` 在补齐输入与源码快照记录时主动中断，
只含部分试运行，**不是冻结 evidence**。其后 `_frozen` 完成了 30 配置，但发现
Windows Git 元数据读取编码警告；修正读取编码后按同一配置重跑至 `_verified`，
后续 stage 仅使用 `_verified` 的同版本源码。所有早期目录保留，不回填覆盖。
数据文件仍按 .gitignore 留在 artifacts，不上传 GitHub；Git 跟踪配置、工具、测试和报告。
