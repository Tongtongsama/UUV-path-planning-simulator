# 3DoF 动态展示版 v1 — 离线历史回放

2026-09-30。算法、配置和实验成功率保持冻结；本轮只读取证据，不调用 Planner、
Navigator、PID 或 Physics，不重跑任何闭环。ROS 2 接口排在本展示版之后。

## 直接观看

在 Edge/Chrome 中打开 [通用播放器](../artifacts/replay_display_v1/index.html)，无需服务器、
网络、Python 或 npm。数据已嵌入一个 HTML 文件，可复制该文件到另一台电脑离线播放。
页面左上角有五个场景入口，也可使用地址末尾的 fragment：

| 场景 | 入口 |
| --- | --- |
| double_baffle（默认） | `index.html#double_baffle_detour` |
| horizontal | `index.html#horizontal` |
| detour | `index.html#detour` |
| edge_forced 碰撞 | `index.html#edge_forced_detour` |
| zigzag 碰撞 | `index.html#zigzag_maze_detour` |

[double_baffle 对照视频](../artifacts/replay_display_v1/double_baffle_comparison_4x.webm)：
1280×1020，WebM/VP9，共同物理时间 4× 播放，末尾短暂停留；不是两臂各自归一化时间。
播放器/视频中的流畅画面不代表预测策略可以实时运行。视频已在浏览器解码并抽查 25 s
附近画面：约 t=100 s 时 approach 固定在最终 SUCCESS 状态，P1 仍继续原轨迹。

## 交互与数据语义

- 主画面：ENU x-z、原矩形障碍、边界/侵蚀边界、S/G、固定 P1 路径、已执行轨迹。
- UUV 标记随实际 pitch 旋转，深色圆点标艇首；艇形放大，仅为姿态示意。
  虚线圆按 vehicle radius + safety margin 绘制，不等同于艇形轮廓。
- 状态栏显示原始样本的时间、phase、对地平移速度、pitch、q、到目标距离，以及日志中
  全程最小障碍净余量。边界显示不能代替连续安全检查，7.2 mm 不靠像素判别。
- 播放、暂停、拖动、原始采样时间单步、0.5/1/2/4/8×、重新播放，共用一个时钟。
- 姿态、q、施加 surge force 与 pitch moment 四条同步曲线，两臂叠加及共同游标。
  pitch 保留原始连续值，不用 wrapping 隐藏绕圈；控制量零阶保持。
- 位置/姿态插值仅用于动画和曲线光标，数值栏显示游标之前最近的原始状态；不导出
  插值状态为实验数据。实验臂结束后固定最终状态，曲线不向未来延长。
- phase 采用每一步日志，而非按位置猜测。COLLISION 标记在该步末检测时刻，保留
  `[step_start, step_end]` 区间，不声称知道步内精确接触时刻；该臂不再运动。
- double_baffle 改善来自 approach；capture 最后为 ORIGINAL_SETTLE，不伪称发生位置修正。
- 五个场景都来自同一零流、0.2 m/s 的 P1 / APPROACH 数据；不混入 P2b 或原 LOS 16 场景。

## 原始证据绑定

每臂使用 `artifacts/terminal_approach_full_v1/runs/<scene>_<arm>_1.json`。
导出前核验完整运行 manifest。路径来自 `artifacts/planner_paired_v1/geometry.json`，
同时校验其 manifest 和 terminal freeze 记录的几何哈希。障碍/边界来自两份
`config/scenarios/planner_acceptance_v{1,2}.json`，哈希必须匹配当时冻结源码。
仅为 horizontal 选择既有的无障碍场景，起点、终点、约束仍来自冻结 v1 配置。

交付目录：`artifacts/replay_display_v1/`。包含自包含页面、无截断精度的状态/控制数据、
来源索引、6 项专项测试 JUnit、浏览器验收、截图、视频以及 `delivery_manifest.json`。
最终 manifest SHA-256：
`536843217908e12009c6dc4c3e67bcabea2f629ef98f9bc309bdaaac821de084`。
`artifacts/replay_v1/` 是早期预览目录，不是最终冻结入口；原仿真 artifacts 没有修改。

## 复现（不重跑闭环）

在仓库根目录运行，输出目录必须尚不存在：

```powershell
.\.venv\Scripts\python.exe -m visualization.replay_export --output artifacts/replay_new
.\.venv\Scripts\python.exe -m pytest tests/test_replay_export.py -q --junitxml=artifacts/replay_new/replay_tests.xml
Start-Process artifacts/replay_new/index.html
```

可选浏览器 QA / 视频生成：需要 Node、Playwright（本轮 1.62.1）和已安装的 Edge。
`NODE_PATH` 应指向包含 playwright 的 node_modules；播放器本身不需要这些依赖。

```powershell
node scripts/test_replay.cjs artifacts/replay_new --video
.\.venv\Scripts\python.exe -m visualization.replay_finalize --output artifacts/replay_new
```

验收包括：五场景选择、播放/暂停、倍速、拖动/重播、单步、共同时间、结束冻结、
碰撞冻结、71 s approach 阶段、窄屏不横向溢出、无浏览器运行错误、视频实际解码。
本轮 **6 个新专项测试通过**；保留原算法 **847 passed** 记录，但未声称重新执行全仓测试。
动态展示只提升可观察性，不新增实验成功率、安全保证或实时性能结论。
