# Dissertation 统一证据索引

核对日期：2026-11-15。本轮仅阅读、哈希核验和对既有状态做代数/时间检查；没有生成新动力学轨迹、启动 ROS/Gazebo/xdyn 或修改冻结文件。

## 结论与版本解释

- G12 参考 Python：54/54，通过九组重复性。真实 GUI 矩阵：54 条实际运行，53 通过、1 失败；45 名义条目 44/45，九次重复自身验收通过，九组原 N6/R2 通过。N6 是每组至少 4/5、P0 成功且无安全违规，不能写成真实 54/54。
- 失败 S1/F1/P4：反馈停滞、MAS 清理失败及关停原生崩溃；未完成任务。起始原因未证实，不归为已证明的动力学不可行；原失败不可被后续复现替换。
- S3 保持 double_baffle_tight_v1 名义场景，本轮真实 18/18。历史 feasible_v2 全高 S2 真实 18/18 也有效，但不拼入 G12 矩阵。
- 有限评价：三档强流 Python 失败、真实门禁拒绝；两档阻尼 Python 和 GUI xdyn 均通过。不是五项真实全部运行/通过。
- 最低交付尚未完成：干净环境复现、最终图表、可移植发布包与完整 dissertation 仍缺。

范围依据：[范围基线](dissertation_scope_baseline_v1.md)、[数值基线](dissertation_model_acceptance_v1.md)。二者历史的“尚未实现”、模型 v1、参考俯仰 60°和巡航 0.4 m/s不能作为最终配置描述。实际模型是 uuv_twin_axial_v2；feasible_v2/G12 独立修订记录了参考限幅 70°、G12 巡航 0.48 m/s、lookahead 2.3，其余共同配置以冻结 release 和 plan 为准。实际姿态 75°、安全 0.9 m、RMS 0.35 m、最大误差 0.8 m 等验收阈值没有因此改变。论文须交代版本演进，不能称所有历史设计参数从未变化。

注意命名冲突：范围文档 C1–C6 是交付标准；数值基线表中的 C1/C2/C3 是速度阶跃、俯仰阶跃、断流控制试验。论文建议分别写“交付 C1”和“控制阶跃 C1”。

## 证据路径字典

以下路径相对项目根 `C:/Users/Songyuan/uuv_simulator`，均在本轮检查中实际读取。索引文件自身不是代替原始数据的新验收器。

|键|主要入口及原始数据|
|---|---|
|E01|`artifacts/twin_axial_v2/suite_recheck_20261031/suite.json`；九个子目录的 `acceptance.json`、`transactions.jsonl`、`manifest.json`|
|E02|`artifacts/twin_axial_v2_numerics/recheck_20261031/acceptance.json`、`protocol.json`、三份 `states_*.json`、`dt_*/transactions.jsonl`、`manifest.json`|
|E03|`artifacts/twin_axial_navigation/current_m6_20261031/report.json`；F1/F2 子目录及 manifest|
|E04|`artifacts/twin_axial_g12_matrix/python_20261110_01/matrix.json`、`environment.json`、`source.json`、54 条 states/report/manifest|
|E05|`artifacts/twin_axial_g12_real_matrix_20261113_01/final_audit_20261114.json`；`result_01…54_*.json`、九份 `group_S*_F*.json`、54 条原始目录|
|E06|`artifacts/twin_axial_g12_boundary_20261114_03/final_audit_20261115.json`、`real_summary.json`、`real/D80`、`real/D120`；五份 Python 记录来源为 `_20261114_02/python`，复制来源由 `python_copy_provenance.json` 绑定|
|E07|`ros2/twin_axial_g12_link_v6/release_01.json`；`ros2/twin_axial_g12_boundary_v3/release_01.json`；其递归依赖的旧 release 不可丢弃|
|E08|`artifacts/dissertation_closeout_20261115_01/read_only_evidence_audit.json`、`inspected_files_sha256.json`、`current_environment_inventory.json`|
|E09|`docs/lotusim_twin_axial_v2.md`、`docs/lotusim_twin_axial_current_f1_f2.md`、`docs/lotusim_twin_axial_s3_transient_v2.md`、`docs/lotusim_twin_axial_g12_boundary_20261115.md`|

E02/E01 文档里旧 `validation_20261026_01` 路径在现工作区不存在，但历史目录 `C:/Users/Songyuan/uuv_simulator_history/lotusim_minimal_dynamics_20261028` 中保留。现有 recheck 的主结果与历史报告相同不代表源码快照或环境相同；优先引用本地 recheck，并保留各自 manifest/来源。历史定位表见 E05 同目录 `c1_evidence_locations.json`。旧文档中的 `/mnt/c/Users/刘松原/` 也不是当前路径。

## C1–C6

|交付标准|证据、当前判断|尚缺/限制|对最低交付的影响|
|---|---|---|---|
|C1 模型一致性|E01–E03、E08；合成非线性六自由度 xdyn，验证限于纵向任务；执行器秩2、Z输入0|将模型方程/坐标/执行器及下表写入论文；M3 区分瞬时理论值与有限步测量|不能声称真实艇型准确度、任意六自由度或所有恒流已验证|
|C2 环境配置|E04/E05：三场景、五初态、三流，场景身份和实际位姿回写检查；E06 另列扰动配置|发布包内固定配置路径和复现步骤|主要为交付整理，不需要扩大场景|
|C3 动力学闭环|E05 指令/分配/实际输入/状态闭环；E06 两条也完整通过|历史单次反馈/清理故障未充分归因；新环境运行可靠性未验证|应披露并在复现观察，不能称零故障|
|C4 名义到达|E04 54/54；E05 53/54、九组 N6/R2；S3 18/18|原失败保留；组通过不等于逐条全通过|满足既定组统计口径，不自行提高或降低完成标准|
|C5 对照/边界|E05 名义对照，E06 三强流参考失败、两阻尼真实通过及零饱和|强流真实未运行；边界为采样点而非连续临界值；整理归因图表|现有有限评价可用于论文，不能称五项真实完成；若交付要求所有压力项真实执行，该项仍未满足|
|C6 复现/论文|release、原始日志、哈希及分阶段报告已具备|干净环境构建/复现、依赖取得可用性、可移植命令、最终图表、发布包、完整论文|仍是最低交付的主要未完成项|

## M1–M7

|模型标准|核验与定位|状态及证据边界|
|---|---|---|
|M1 分配/欠驱动|`ros2/twin_axial_v2/model.py`、`allocation.py`、`tests/test_twin_axial_v2.py`；E01/E05 allocation/limits/no_direct_heave；E08 静态矩阵 `[[1,1],[0,0],[-.25,.25]]` 秩2|实际输入重构和限幅已有通过证据。被动附加质量 Z 力不等于直接执行器|
|M2 静止30 s|E01 `zero/acceptance.json`：600 步；zero_pose_drift、zero_attitude_drift、neutral_rest 通过|无流水平静止，仅此条件|
|M3 零态响应|E01 正负 surge/pitch 的 initial_acceleration；E08 首有效步重新计算：u_dot 约 ±0.07968064，对 ±0.08 相差0.3992%；q_dot约 ±0.04961436，对±0.05相差0.7713%|均在1%内，但这是 dt=.05 的有限差分，不是求解器瞬时导数输出。零态 w_dot=0 由无耦合/阻尼/恢复项且 Z输入0的方程得到；未找到独立求解器瞬时 w_dot 原始输出，不把方向测试冒称该输出|
|M4 能量|E02 energy：无流撤力后10 s，xdyn dt=.05 的能量比约0.038279，各步长通过|含恢复势能；不要求每个速度分量单调|
|M5 收敛|E02 convergence：.05→.025 的位置终点差2.73e-10 m，俯仰1.89e-7°，速度1.91e-10 m/s；继续减半通过|固定输入20 s，带浮点噪声下限；非所有任务全局收敛证明|
|M6 配对|E02 consistency/coupling_diagnostic 无流通过；E03 F1/F2 固定输入20 s通过|相同方程的参考/求解器一致性，不是实艇验证；强流及扰动模型不能直接继承“已配对20 s”的结论|
|M7 坐标/时间|E01时间连续/姿态回写、v2 protocol；E08 对1880个既有有效状态反变换：位置0 m、旋转矩阵最大差4.53e-16、dt误差7.08e-16 s、四元数范数误差2.22e-16|满足记录样本的原限值；本轮只做代数检查，没有重新积分，也不声称任意姿态全覆盖|

E08 同时检查了 E01 九条记录的 manifest，未发现不匹配；170 个最终边界发布绑定文件核对通过。本轮没有重新运行历史数值验证或全量测试，旧测试数量仅能归属旧报告。

## 论文证据组织与明确缺项

建议章节对应：方法/RQ1→C1/M1–M7；系统集成/RQ2→C2/C3；结果→C4和九组重复性；讨论/RQ3→C5、强流拒绝、S3窄裕量、工程失败；复现附录→C6。

必须列出的未完成项：①干净环境实际复现；②依赖/运行文件可取得性和发布包闭包；③最终图表与完整正文；④历史链路失败根因仍未证实；⑤M3证据口径解释；⑥强流真实未执行。不要为消除这些文字缺项直接调阈值、重跑覆盖失败或增加未经登记的研究工况。

执行准备见 [干净环境复现清单与命令](dissertation_clean_reproduction_20261115.md)。
