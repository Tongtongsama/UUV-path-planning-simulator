# G12 real v3：时间边界、IPC 与证据门禁的离线修订

日期：2026-11-11。独立入口 `ros2/twin_axial_g12_real_v3`，旧 v2 原型、旧 release、旧记录均保留。

## 修改与不变量

- watchdog：仅在0–301秒任务时钟内，把负向不超过4个ULP的表示差视为零。8.1与8.100000000000001的差异由此不再被当作未来命令。最大修正约2.3e-13秒；原0.25秒仿真期限（含已有1e-6数值容差）和1秒墙钟期限保持不变。真正未来的命令仍被拒绝。未修改G12的arming比较或任何控制参数。
- 两侧IPC共用二进制JSONL读取器：只提交完整且解析成功的换行记录，UTF-8半字符/半行等待补齐；完整坏行、文件替换/截断/消失明确拒绝，不静默丢记录。网关解析失败锁存invalid并写ipc_error.txt，watchdog归零。
- 删除启动Gazebo后、开始消费IPC之前的十次阻塞DDS诊断循环。其余原有PID、分配、模型、场景、物理验收和结束协议未重设计。运行器仍必须通过门禁才能创建输出和启动进程。
- 新release绑定旧Python114文件、274个证据文件、旧适配器依赖、全部v2及v3 Python源码。新运行源码快照明确包含v2/v3，记录内复制transport证据；不让旧release冒充新运行版本。
- 不再硬编码transport通过或record binding通过。验收逐文件核对运行快照与新release。旧开发运行不因此重判通过。

## 真实门禁的状态

本轮完成的是严格的证据校验器，不是实测传输通过。门禁先重新核查冻结54条Python原始状态/控制、9组重复性和反馈适配；另要求与当前release、Nix文件及GUI方式一致的真实联合通道证据。无证据、缺通道、哈希不符、源码不同、过期、时序异常或动作失败均拒绝。

当前离线CLI只提供检查，不提供启动命令：

```powershell
.\.venv\Scripts\python.exe -m ros2.twin_axial_g12_real_v3.cli --python-matrix artifacts/twin_axial_g12_matrix/python_20261110_01 --report artifacts/twin_axial_g12_real_v3/check_NEW.json
```

必须选择新报告路径。没有真实证据时退出码2是预期拒绝，不是把离线测试结果伪装成通过。

真实联合探针记录格式（尚未采集，测试中的合成样本不能用于放行）：

- `probe.json`：origin=actual_integration_probe、gui、adapter_release_sha256、runtime_files、completed_unix。
- `events.jsonl`：state/command/pose分别至少30条，sequence连续，发送/接收payload一致、不同PID、单调时钟延迟0–0.25秒；pose实际消息类型为lotusim_msgs/msg/VesselPositionArray。action记录accepted、status=4、非空且全部成功results。termination记录确认zero_applied、清理前后动力学步数相等、cleanup_completed。
- `manifest.json`：上述两个文件的SHA256。探针完成距启动不超过600秒；启动前重新核对证据、源码和运行环境。此时限只限制证据新鲜度，不改变仿真watchdog。

**剩余事项**：现有旧transport_probe没有自动生成这个联合证据格式；下一轮需要在独立、可审计的实际进程联合探针中采集原始事件，验证IPC、Gazebo pose、MAS action和终止流程。不能手写passed或把单元测试fixture用于正式门禁。当前无此实测证据，不能直接启动四组正式复验。结束协议仍需实际验证，本轮不宣称停止/清理时序已修好。

## 测试和证据

新增60项测试覆盖三条真实时间戳回归、截止时间、真正未来命令、顺序/非法时间、UTF-8跨字节半行与多行、坏行/截断/消失、无证明禁止启动、源码漂移、旧记录绑定拒绝、缺失源码、真实通道证据缺项/失败/过期。原物理验收文件与v2逐字节一致。

首次针对性测试有一个Windows路径分隔符导致的测试桩未命中，记录为targeted_01.xml，保留；修正测试后最终全量1128项通过，见offline_20261111_01/regression.xml。

本轮未启动xdyn、LOTUSim、Gazebo、ROS节点或任何新的动态轨迹。54条检查是已有原始记录的离线重算。最终门禁与保全结果见同目录summary.md和decision.json。
