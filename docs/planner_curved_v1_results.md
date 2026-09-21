# P2 collision-aware circular fillet — 首轮结果

2026-09-21。P0/P1 基线已单独提交为 **1a8a105**，未推送。
本实验不替换 P1，不修改 PID、LOS、startup/capture、海流或安全预算。

## 结论

**P2 暂不适合替换 P1。** 两个原碰撞场景没有获得可执行的不同曲线路径；
几何拒绝不能记为避碰成功。detour 虽仍成功，但出现 guidance/采样兼容性退化。

| 每臂 12 场景（重复不算新样本） | P1 | P2 |
| --- | ---: | ---: |
| SUCCESS | 7 | 6 |
| COLLISION | 2 | 0 |
| TIMEOUT | 1 | 0 |
| CAPTURE_FAILED | 2 | 2 |
| STARTUP_FAILED | 0 | 1 |
| UNSMOOTHABLE / UNKNOWN | 0 | 3 |

两臂零流、0.2 m/s、原 120 s 预算，每条件两次独立重复。
总计 **48 份试验记录**：42 次 Navigator 调用（其中两次启动即拒绝），
6 份几何拒绝记录不调用 Navigator。每对 history/状态/指标一致；
全部 24 次 P1 history 与原 P0/P1 evidence 完全一致。全仓 **829 passed**。
不能把 P2 的 6/9 已接受场景成功率与 P1 的 7/12 混比。原 LOS 10/16 不变。

## 几何实现与边界

- `trajectory/circular_fillet.py` 同时分配所有切线区间，拒绝相邻圆角重叠和 180° 折返。
- 固定 **R=1 m**，只是预注册几何候选，不是实测最小转弯半径；没有结果后缩半径。
- 最大弓高 0.001 m、最大弦长 0.1 m。每条弦用
  `required_clearance + sagitta + numerical_epsilon` 连续检验，保守覆盖整弧及圆形 footprint。
  原 required clearance 为 0.3 m；所有直线连接与最终 Path 再次连续验证。
- 底层连接切向连续、曲率在直线/圆弧处跳变；输出是有弦误差界的折线逼近，
  不能声称输出折线本身处处曲率有界，或执行器可实现。
- 固定 lookahead=0.5 m 必须小于每条采样圆弧长度的 80%。
  这**不保证偏离路径后的 LOS 连线安全**，也不保证投影器兼容细密线段。
- 先检查原 P1；不通过时，仅按已冻结成本次序检查原 buffer 集合中的不同路径。
  未生成额外候选或修改 Planner。geometry.json 保存所有检查与拒绝原因。

## 两个碰撞场景：拒绝而非改善

先执行 edge_forced_detour、zigzag_maze_detour，审核之后运行其余十场景。
P1 两次都复现碰撞，侵入约 2.34 mm / 1.82 mm。
P2 均因 `ADJACENT_TANGENT_REGIONS_OVERLAP` 拒绝；固定 buffer 集合只有同一条唯一可用路径。
并未检验“更外侧曲线能否消除碰撞”，也不能声称整个空间物理不可行。
double_baffle 同样几何拒绝；原 timeout 不因此得到解决。

## u_trap：几何候选与启动合同冲突

原 P1 内切圆弧被 `ARC_SWEPT_CAPSULE_UNSAFE` 拒绝。
buffer=0.5 m 候选通过几何检验，但第一段从 (1,-5) 垂直向下到 (1,-8)，
启动目标 -90° 超出现有 70° command envelope，t=0 返回
`startup heading outside configured envelope`。未放宽 guard 或改变初始姿态。
这不是 PID 跟踪失败；几何可行不等于任务入口可执行。

## detour：成功但动态表现变差

| 指标 | P1 | P2 |
| --- | ---: | ---: |
| 结果 | SUCCESS | SUCCESS |
| 路径长度 m | 10.485 | 10.399 |
| 完成时间 s | 63.95 | 72.45 |
| 最小执行障碍净距 m | 0.8643 | 0.2538 |
| 最大实际 q 绝对值 rad/s | 0.383 | 1.387 |
| 最大 wrapped raw LOS pitch step rad | 0.0160 | 0.8129 |
| PATH_FOLLOWING 最大路径偏差 m | 0.233 | 0.885 |
| 力矩饱和持续时间 s | 0 | 7.35 |
| 最大未限幅力矩绝对值 N m | 5.155 | 36.708 |

当前投影器门限 `min(switch_radius, .45*L_i, .45*L_next)` 对约 0.08724 m
弦段收缩为约 **0.03926 m**。事后诊断观察到 committed progress 平台：
33.45–36.35 s（段 5）、42.90–46.20 s（段 10）、46.35–52.70 s（段 12）。
平台末端横向误差仍略大于门限；车辆继续运动时 LOS 参考发生较大变化。
这是**细密采样与顺序 endpoint gate 交互**的证据，不是“圆弧无效”或“须加 PID 增益”的证明。
未修改门限、增益或滤波来掩盖本次结果。上下斜线两臂仍 CAPTURE_FAILED。

## 诊断、测试与证据

`artifacts/planner_curved_report_final` 含全 12 场景路径对照图及各场景的：
有效 pitch reference/实际 pitch、q reference/实际 q、未限幅/公开/施加力矩、P/I/D/FF 分项。
分项从已记录诊断恢复，逐步验证和等于 requested moment，没有改控制输出。
raw LOS q_ref=0 仍是既定占位值，不是角度参考真实导数。
红虚线表示首次观测最小净距时刻；1 s progress 平台是事后诊断而非新拒绝条件。

配置 `config/planner_curved_v1.json`；数据 `artifacts/planner_curved_v1`；
测试 `artifacts/planner_curved_tests_final.xml`；[冻结索引](planner_curved_v1_dataset.json)。
新增 20 项测试，覆盖正负转角、整弧碰撞/弓高余量、重叠、折返、采样预算、退化路径、
参数、力矩分解与小线段门限。工件不入 Git；索引不能替代数据，未声明异地备份。

```powershell
.\.venv\Scripts\python.exe -m validation.planner_curved --stage geometry --output artifacts/planner_curved_new
.\.venv\Scripts\python.exe -m validation.planner_curved --stage collision --output artifacts/planner_curved_new
# 审核两个碰撞场景后运行其余场景；不可覆盖已存在的阶段目录。
.\.venv\Scripts\python.exe -m validation.planner_curved --stage remaining --output artifacts/planner_curved_new
.\.venv\Scripts\python.exe -m validation.planner_curved_report --input artifacts/planner_curved_new --output artifacts/planner_curved_report_new
```

## 下一步

保留 P1 默认与 P2 失败证据。若继续曲线路线，先独立验证采样密度与投影 gate 的
表示兼容性，以及候选 startup 合同，再判断曲线闭环价值；不混入 PID 调参。
TerminalApproachPolicy 可独立推进，但本轮未实现。ROS 2 adapter 不必等待所有失败解决。
