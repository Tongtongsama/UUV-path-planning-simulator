# S3/F2/P2 已记录轨迹可视化

日期：2026-11-16；时区 Asia/Singapore。

## 材料性质

只做已有记录的可视化，**没有启动新的 Gazebo/ROS 运行，没有刷新门禁，没有新录制实时演示**。画面全程标注 `RECORDED REPLAY / 已记录轨迹回放 — NOT LIVE`，不能作为现场真实闭环证据。

本轮没有找到这五条对应的已保存真实运行截图/录像，因此使用保存的 `navigation.jsonl`、`gazebo_poses.jsonl`、`plan.json`、`acceptance.json` 生成回放；不是把回放冒充现场录像。

## 输出

- `artifacts/gui_visualization_20261116_01/S3_F2_P2_recorded_replay.gif`：约 69.7 s，800×450，6 fps，2.5× 回放。
- `artifacts/gui_visualization_20261116_01/contact_sheet.png`：开场、中段、验收/边界三帧总览。
- `artifacts/gui_visualization_20261116_01/boundary_table.png`：历史失败、五条结果、强流拒绝与完整矩阵边界的独立表。
- `artifacts/gui_visualization_20261116_01/visualization_manifest.json`：源/输出哈希、时长、速度、验收值。
- `artifacts/gui_visualization_20261116_01/README.md`：材料说明与限制。
- `artifacts/gui_visualization_20261116_01/render.py`：逐帧 PNG 与 GIF 生成脚本。

## 演示结构

1. **约 8 s**：S3/F2/P2、`double_baffle_tight_v1`、P2 初态、0.05 m/s、冻结 G12/v6、xdyn/LOTUSim/Gazebo 标签；明确“非现场实时闭环、未新录”。
2. **约 50 s**：已有轨迹回放，2.5× 加速；艇运动、规划路径、障碍、净距 0.9 m 阈值、路径误差 0.8 m 阈值同步；当前时间/净距/误差随帧更新。
3. **约 12 s**：原验收 PASS、关键数值、日志目录；明确列出 S1/F1/P4 历史失败、P2 仅 0.0162 m 净距裕量、强流真实拒绝和五条不等于完整矩阵。

## 原验收

S3/F2/P2：到达 129.65 s；最小净距 0.91617 m；路径 RMS 0.33000 m；最大误差 0.72625 m；饱和 0；位姿覆盖 100%；28 项检查全 true。净距相对 0.9 m 阈值仅余 0.01617 m，属于最紧工况，不能外推所有参数。

## 必须保留的边界

- S1/F1/P4 历史是失败（`State feedback stale`）；新五条中的 S1 通过是独立新记录，不覆盖历史失败。
- 强流 Python 失败、真实门禁拒绝的结论不变。
- 五条 GUI 是最小复现，不是完整 54 条真实矩阵，也不是九组重复性的重新验证。
- 本材料没有新录实时演示；若需要新录像，应另行登记、刷新门禁，不属于本整理任务。
