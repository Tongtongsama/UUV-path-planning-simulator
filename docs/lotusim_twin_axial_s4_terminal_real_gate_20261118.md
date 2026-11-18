# S4 4 m终点接近候选：独立真实适配与联合门禁

日期：2026-11-18。适配器 ros2/twin_axial_s4_terminal_real_v1/；本轮记录 artifacts/twin_axial_s4_terminal_real_gate_20261118_01/。本轮只允许离线核验及短GUI联合探针，不启动正式导航。

## 冻结边界与实现

复用冻结 v6 IPC、gateway、时序、PID、分配器、xdyn接口、正常/异常清理及原验收。未引入同日另一研究修订 current_v2 的时变水流逻辑。本任务仅F0/F1/F2原恒流。

独立 mission 将已验证候选路径送入原真实 Mission：继承原 update 和 Guide，仅替换计划生成与S4场景构造。SDF保留原艇体和GUI模板，生成全部五个原S4矩形，逐项核对其边界。候选规划函数直接复用，冻结模型 model_config 直接复用。

trial.py 从原v6运行器派生，只修改任务适配导入、独立门禁和五条工况限制；仍调用原v6 gateway与IPC。acceptance.py 只修改Mission导入，单条验收逻辑及阈值保持。没有实现S4完整真实矩阵验收扩展；只支持本轮指定的五条单条验收。

允许范围仅：S4/F0/P0、S4/F1/P0、S4/F2/P0、S4/F2/P2、S4/F2/P4。启动必须使用独立S4类型的proof并在实际运行前guard；原v6或普通JSON证明不能直接代替。试验目录包含独立release、源码快照及三组联合证据拷贝。

## Python与适配等价性门禁

核对 terminal_validation_20261117_01 的完整最终清单、源码快照、三条P0原始来源、全部18条身份/配置/路径/source_sha256及原验收。所有18条重新进行离线动力学/引导/PID/分配核验；三组严格重复性重新计算。

对18条原始状态逐步驱动新真实Mission，比较导航字段、速度/俯仰目标及到达时刻。该操作不积分新轨迹、不启动ROS，不属于新增Python试验或真实闭环。21,454步离线适配核验通过见 offline_check.json。

release_01.json绑定冻结v6 release、候选Python最终证据清单及全部依赖/新增源码。未来guard重新核对文件和探针完整性、Nix环境及新鲜度；不根据已保存的passed JSON跳过运行前检查。

## 联合探针的准确范围

复用原v6 normal、故意abort、repeat探针，使用原S3/F0/P0零输入测试平台，验证共享IPC状态/指令实际交付、真实Gazebo位姿类型与匹配、MAS请求/应答、归零、停止积分和退出协议。正常探针为13 s仿真时间。故意abort记录预期失败，验收的是异常清理成功，不能改写为普通成功试验。

这些探针不是S4导航，也不证明五障碍中的航行通过。S4的地图及适配等价性由离线证据检查，实际导航仍留待另行授权的五条小试。新门禁将两部分绑定，不能把原S3探针称为S4真实验收。

使用独立目录；探针只终止自己启动的进程组，不关闭已有Gazebo。每次开始前记录端口状态，最多等待90 s自然释放；不杀占用端口的外部进程。normal/abort/repeat任何检查失败都保留并停止本次门禁编排。

## Nix与时效

通过锁定的 /home/nixos/.cache/uuv-twin-axial-g12-real-nix、nix develop --offline --no-write-lock-file 执行。核对运行文件、flake.lock、ROS类型支持、GUI显示/驱动、端口。repeat证据600 s新鲜度保持不变；检查时通过不构成永久许可，正式启动前仍须guard，过期需重采集并刷新门禁。

本轮命令为新记录目录 run.sh，仅调用原v6 probe与独立gate，没有调用 trial.run_trial。未来正式运行入口为独立 trial.run_trial，必须提供GUI、登记工况、独立输出目录及三组探针参数，经内部完整gate/guard后才能启动。

最终状态以本目录summary.md、gate.json、decision.json或failure.json为准。原真实53/54、九组验收通过、五条GUI复现通过、旧B六条F2失败及强流真实门禁拒绝结论不变。

## 本轮首次拒绝与独立刷新

_01 的 normal、abort、repeat 均通过，S4与基础Python证据也通过，但真实门禁的运行环境快照记录12356端口不可绑定，故 passed=false、ready_to_run=false，未调用正式导航。原gate.json和decision.json完整保留。随后只读检查未见监听进程；不能仅据此把具体端口状态原因认定为已证实。

独立 _02 目录仅补采新的repeat，复用本轮_01原始normal与abort，并在门禁检查前显式等待12356/12357自然可绑定（最多90 s）。这不改变门禁阈值、不杀进程、不覆盖第一次拒绝、不修改冻结适配器release。按照原v6规则，实际启动时至少repeat须在600 s内，其余两组仍须通过完整原始证据核验。第二次结果以 artifacts/twin_axial_s4_terminal_real_gate_20261118_02/ 的记录为准。

## 最终运行条件：未放行

_02重复探针在启动前再次被端口bind拒绝，原始异常与后续缺少manifest的堆栈均保留。_03在探针前、门禁前等待无TCP占用且连续五次可绑定；新repeat通过。最终采用_01/normal、_01/abort、_03/repeat的原始证据组合，所有协议检查、候选18条核验及21,454步等价性均通过。

但_03完整证据复查期间，另一个 `gui_current_v3_20261118_01/S3/F2/P0` 实验占用了12356/12357，并运行其Gazebo。最终runtime检查两端口均false，总passed=false、ready_to_run=false；正式导航未启动，没有关闭该实验或任何用户已有Gazebo。

因此适配及证据工作已完成，当前不能放行五条GUI小试。应等另一实验自然结束，并为完整门禁及小试安排独占两端口的时间窗，再按原600 s新鲜度刷新证据与guard；不能修改冻结控制或放宽门禁来绕过资源冲突。最新结论及证据见 artifacts/twin_axial_s4_terminal_real_gate_20261118_03/readiness_conclusion.md、gate.json、resource_conflict.json；前三次全部保留。全量回归1,249项通过，冻结271项release依赖核对保持。
