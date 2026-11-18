# S4 单一终点接近规划候选 v1

日期：2026-11-17。用户授权范围仅一个候选、几何检查和三流 P0 Python 小试；任一失败即停止。没有真实仿真入口。

## 固定设计

候选 ID：S4_B_terminal_straight_v1。独立入口：ros2/twin_axial_s4_terminal_candidate_v1/。结果目录：artifacts/twin_axial_s4_terminal_candidate_20261117_01/。

保留已有 A*（0.5 m、8邻接）、安全捷径及共线点消除；先从原起点 (2,-10) 规划到中间几何约束点 (14,-10)，对这一前缀使用原安全捷径，再追加原目标 (18,-10)。最终 4 m 为水平段，不对整体再做可能跨过中间点的捷径。无新增圆角、无新速度规划、无控制参数修改。全路径按最大 0.05 m 间距保留顶点细分，并以原 1.5 m 净距逐段校验。

中间点不是新到达目标，没有停靠、保持或控制状态切换；唯一到达仍为 (18,-10) 的原 0.5 m 半径和 2 s 保持。S4 障碍、边界、恒流及 P0 不变；冻结 G12、模型、PID、分配器和所有验收限值不变。

设计假设：将垂向位置调整提前，为最终接近提供较长的方向稳定段，从而改善 F2 进入目标区时的状态。该假设未经本轮结果预判；水平段也可能使速度升高，不能据此保证通过。

## 预登记和停止规则

preregistration.json 在候选规划和动态结果产生前创建，包含精确几何规则、参数、源码清单绑定。按 F0 → F1 → F2 顺序运行，各一次 P0，三流使用同一路径。几何拒绝、任何原验收失败、离线验证失败或入口异常均立即停止后续水流，不重试、不调整候选、不替换记录。未运行条目明确标注 not_run，不能记为动态失败或通过。

直接调用冻结 G12 matrix runner；独立作用域只提供候选路径和原 S4 场景。每条运行后复用已有离线核验器，重算误差、净距、到达保持、包络和分配，并逐步核对动力学、引导及 PID。原 S1–S3 专用稳态证书不适用于 S4；这不等于真实门禁通过。

只有三条原验收及离线核验全部通过，才建议后续另行授权剩余 15 条；本轮不自动扩展。旧 B 的 12/18 与六条 F2 失败保持，不拼入新候选验收。

## 复现命令

```powershell
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_s4_terminal_candidate_v1.cli register artifacts/<新的独立目录>
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_s4_terminal_candidate_v1.cli run artifacts/<同一目录>
```

结果文件拒绝覆盖，禁止使用已有正式结果目录重复执行。输入和冻结源码以 SHA-256 核对；旧实验普通文件核对字节哈希，WSL 链接核对稳定元数据，不宣称验证无法解析的目标内容。

## 结果入口

正式目录的 plan.json、geometry_check.json、各 flow/P0/report.json 与 states.jsonl、各 flow_verification.json、decision.json、preservation.json 和 summary.md 为最终证据。此前真实 53/54、九组验收通过、五条 GUI 复现通过和强流真实门禁拒绝结论全部保持，不由本轮新规划证据替换。
